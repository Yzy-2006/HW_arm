"""Bounded microphone capture, speech filtering and local Whisper inference."""
from collections import deque
from dataclasses import dataclass, fields
from math import ceil, gcd
import time
import math
import re

import numpy as np

from .commands import normalize, number


@dataclass(frozen=True)
class AudioSettings:
    input_device_name: str = ""
    input_device_index: int = -1
    sample_rate: int = 0
    channels: int = 1
    chunk_ms: int = 30
    rms_threshold: float = 0.015
    silence_sec: float = 0.6
    min_speech_sec: float = 0.25
    max_utterance_sec: float = 8.0
    pre_roll_sec: float = 0.25
    model_name: str = "base"
    device: str = "cpu"
    language: str = "zh"
    backend: str = "whisper"
    compute_type: str = "int8"
    cpu_threads: int = 4
    beam_size: int = 5
    vad_filter: bool = True
    initial_prompt: str = ""
    max_text_chars: int = 32
    min_avg_logprob: float = -0.8
    max_no_speech_prob: float = 0.6
    max_compression_ratio: float = 2.4

    @classmethod
    def from_dict(cls, raw):
        unknown = set(raw) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"未知 audio 配置项: {sorted(unknown)}")
        settings = cls(**raw)
        for name, minimum, maximum in (("input_device_index", -1, 65535),
                                       ("sample_rate", 0, 192000),
                                       ("channels", 1, 2), ("chunk_ms", 10, 100),
                                       ("cpu_threads", 1, 64), ("beam_size", 1, 10),
                                       ("max_text_chars", 4, 256)):
            value = getattr(settings, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"audio.{name} 必须是整数")
            number(value, name, minimum, maximum)
        if 0 < settings.sample_rate < 8000:
            raise ValueError("sample_rate 应为 0（自动）或至少 8000")
        for name in ("input_device_name", "model_name", "device", "language",
                     "backend", "compute_type", "initial_prompt"):
            if not isinstance(getattr(settings, name), str):
                raise ValueError(f"audio.{name} 必须是字符串")
        if any(not getattr(settings, name).strip() for name in ("model_name", "device", "language")):
            raise ValueError("model_name、device 和 language 不能为空")
        if settings.backend not in ("whisper", "faster_whisper"):
            raise ValueError("backend 必须为 whisper 或 faster_whisper")
        if not settings.compute_type.strip():
            raise ValueError("compute_type 不能为空")
        if not isinstance(settings.vad_filter, bool):
            raise ValueError("vad_filter 必须是布尔值")
        number(settings.min_avg_logprob, "min_avg_logprob", -10, 0)
        number(settings.max_no_speech_prob, "max_no_speech_prob", 0, 1)
        number(settings.max_compression_ratio, "max_compression_ratio", 1, 10)
        number(settings.rms_threshold, "rms_threshold", 0.000001, 1.0)
        number(settings.silence_sec, "silence_sec", 0.1, 10)
        number(settings.min_speech_sec, "min_speech_sec", 0.01, 10)
        number(settings.max_utterance_sec, "max_utterance_sec", 0.2, 60)
        number(settings.pre_roll_sec, "pre_roll_sec", 0, 2)
        if settings.min_speech_sec + settings.silence_sec > settings.max_utterance_sec:
            raise ValueError("max_utterance_sec 必须 >= min_speech_sec + silence_sec")
        return settings


def select_input_device(devices, index, name, default_index):
    inputs = [d for d in devices if d["maxInputChannels"] > 0]
    if index >= 0:
        candidates = [d for d in inputs if int(d["index"]) == index]
    elif name:
        candidates = [d for d in inputs if name.casefold() in d["name"].casefold()]
    else:
        candidates = [d for d in inputs if int(d["index"]) == default_index]
    if len(candidates) != 1:
        available = ", ".join(f"{int(d['index'])}: {d['name']}" for d in inputs)
        raise ValueError("无法唯一选中输入设备，请指定 input_device_index 或更完整的名称。"
                         f" 可用输入设备: {available or '无'}")
    return candidates[0]


class Segmenter:
    """Waiting silence has bounded memory; duration starts at the first loud frame."""
    def __init__(self, settings, rate, chunk_samples):
        self.settings, self.rate = settings, rate
        self.pre = deque(maxlen=ceil(settings.pre_roll_sec * rate / chunk_samples))
        self.reset()

    def reset(self):
        self.active = False
        self.frames = []
        self.speech = self.silence = self.total = 0
        self.pre.clear()

    def feed(self, frame):
        frame = np.asarray(frame, dtype=np.float32)
        loud = float(np.sqrt(np.mean(frame ** 2))) >= self.settings.rms_threshold
        if not self.active:
            if not loud:
                self.pre.append(frame)
                return None
            self.active = True
            self.frames = list(self.pre)
            self.pre.clear()
        self.frames.append(frame)
        self.total += len(frame)
        self.silence = 0 if loud else self.silence + len(frame)
        if loud:
            self.speech += len(frame)
        if (self.silence < self.settings.silence_sec * self.rate
                and self.total < self.settings.max_utterance_sec * self.rate):
            return None
        result = (np.concatenate(self.frames)
                  if self.speech >= self.settings.min_speech_sec * self.rate else None)
        self.reset()
        return result


def to_whisper_audio(samples, rate):
    if rate == 16000:
        return np.asarray(samples, dtype=np.float32)
    from scipy.signal import resample_poly
    factor = gcd(rate, 16000)
    return resample_poly(samples, 16000 // factor, rate // factor).astype(np.float32)


def assess_transcript(result, settings):
    """Reject unreliable ASR as a whole, never trim noise into a valid command."""
    text = result.get("text", "").strip()
    clean = normalize(text)
    if not clean:
        return text, "no_speech"
    if re.search(r"(.{1,12})\1{3,}", clean):
        return text, "repetitive_text"
    if len(clean) > settings.max_text_chars:
        return text, "text_too_long"
    segments = result.get("segments", [])
    if not segments:
        return text, "missing_confidence"
    for segment in segments:
        for key, limit, above, reason in (
            ("avg_logprob", settings.min_avg_logprob, False, "low_confidence"),
            ("no_speech_prob", settings.max_no_speech_prob, True, "no_speech"),
            ("compression_ratio", settings.max_compression_ratio, True, "repetitive_text"),
        ):
            value = segment.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                return text, "missing_confidence"
            if (above and value > limit) or (not above and value < limit):
                return text, reason
    return text, "accepted"


def load_recognizer(settings):
    if settings.backend == "faster_whisper":
        from faster_whisper import WhisperModel
        return WhisperModel(settings.model_name, device=settings.device,
                            compute_type=settings.compute_type,
                            cpu_threads=settings.cpu_threads, num_workers=1)
    import whisper
    return whisper.load_model(settings.model_name, device=settings.device)


def transcribe_audio(model, samples, settings):
    options = dict(language=settings.language, task="transcribe",
                   condition_on_previous_text=False, temperature=0.0,
                   beam_size=settings.beam_size,
                   initial_prompt=settings.initial_prompt or None,
                   compression_ratio_threshold=settings.max_compression_ratio,
                   log_prob_threshold=settings.min_avg_logprob,
                   no_speech_threshold=settings.max_no_speech_prob)
    if settings.backend == "faster_whisper":
        segments, info = model.transcribe(
            samples, **options, vad_filter=settings.vad_filter,
            vad_parameters=dict(threshold=0.5, min_speech_duration_ms=250,
                                min_silence_duration_ms=200, speech_pad_ms=300),
            max_new_tokens=64)
        segments = list(segments)  # CTranslate2 inference runs while iterating.
        return dict(text="".join(s.text for s in segments),
                    segments=[dict(avg_logprob=s.avg_logprob,
                                   no_speech_prob=s.no_speech_prob,
                                   compression_ratio=s.compression_ratio) for s in segments],
                    speech_duration=info.duration_after_vad)
    # OpenAI Whisper and faster-whisper use different names for this argument.
    options["logprob_threshold"] = options.pop("log_prob_threshold")
    return model.transcribe(samples, **options,
                            fp16=settings.device.startswith("cuda"))


def run_microphone(settings, stop, emit):
    """Own audio resources; emit(kind, text_or_rejection, capture_timestamp)."""
    audio = stream = None
    try:
        try:
            import pyaudio
        except ImportError as exc:
            raise RuntimeError("缺少 PyAudio。请按 README 安装到 ROS 的 Python 环境。") from exc
        audio = pyaudio.PyAudio()
        devices = [audio.get_device_info_by_index(i) for i in range(audio.get_device_count())]
        try:
            default = int(audio.get_default_input_device_info()["index"])
        except OSError:
            default = None
        info = select_input_device(devices, settings.input_device_index,
                                   settings.input_device_name, default)
        index = int(info["index"])
        rate = settings.sample_rate or int(info["defaultSampleRate"])
        if not audio.is_format_supported(rate, input_device=index,
                                         input_channels=settings.channels,
                                         input_format=pyaudio.paInt16):
            raise ValueError("声卡不支持配置的采样率/声道数")
        emit("status", f"加载 {settings.backend} {settings.model_name} ({settings.device})；首次运行可能下载模型", time.monotonic())
        model = load_recognizer(settings)
        if stop.is_set():
            return
        chunk = round(rate * settings.chunk_ms / 1000)
        segmenter = Segmenter(settings, rate, chunk)
        stream = audio.open(format=pyaudio.paInt16, channels=settings.channels, rate=rate,
                            input=True, input_device_index=index, frames_per_buffer=chunk)
        emit("status", f"正在监听: [{index}] {info['name']}，{rate} Hz，{settings.channels} 声道", time.monotonic())
        while not stop.is_set():
            # Report overflows instead of treating old buffered audio as fresh speech.
            data = stream.read(chunk, exception_on_overflow=True)
            frame = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            if settings.channels > 1:
                frame = frame.reshape(-1, settings.channels).mean(axis=1)
            utterance = segmenter.feed(frame)
            if utterance is None:
                continue
            captured_at = time.monotonic()
            # Pause capture during inference so queued microphone audio cannot run later.
            stream.stop_stream()
            stream.close()
            stream = None
            result = transcribe_audio(model, to_whisper_audio(utterance, rate), settings)
            text, reason = assess_transcript(result, settings)
            if reason == "accepted" and not stop.is_set():
                emit("transcript", text, captured_at)
            elif not stop.is_set():
                emit("rejected", {"reason": reason, "text": text[:80]}, captured_at)
            if not stop.is_set():
                stream = audio.open(format=pyaudio.paInt16, channels=settings.channels,
                                    rate=rate, input=True, input_device_index=index,
                                    frames_per_buffer=chunk)
    except Exception as exc:
        if not stop.is_set():
            emit("error", f"语音线程停止: {type(exc).__name__}: {exc}", time.monotonic())
    finally:
        try:
            if stream is not None:
                stream.close()
        finally:
            if audio is not None:
                audio.terminate()


def list_devices_main():
    try:
        import pyaudio
    except ImportError:
        raise SystemExit("请先安装 python3-pyaudio，见本包 README。")
    audio = pyaudio.PyAudio()
    try:
        print("index | max input channels | default sample rate | name")
        for index in range(audio.get_device_count()):
            info = audio.get_device_info_by_index(index)
            if info["maxInputChannels"]:
                print(f"{index:5} | {info['maxInputChannels']:18} | "
                      f"{info['defaultSampleRate']:19.0f} | {info['name']}")
    finally:
        audio.terminate()
