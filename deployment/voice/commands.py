"""Validated command configuration and deterministic, ROS-independent matching."""
from dataclasses import dataclass
import math
from pathlib import Path
import unicodedata

import yaml


def normalize(text):
    text = unicodedata.normalize("NFKC", text).casefold()
    return "".join(c for c in text if not c.isspace()
                   and not unicodedata.category(c).startswith("P"))


def number(value, label, minimum=0.0, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是数字")
    if not math.isfinite(value) or value < minimum:
        raise ValueError(f"{label} 必须是 >= {minimum} 的有限数字")
    if maximum is not None and value > maximum:
        raise ValueError(f"{label} 必须 <= {maximum}")
    return float(value)


def string_list(value, label, allow_empty=False):
    if not isinstance(value, list) or (not value and not allow_empty):
        raise ValueError(f"{label} 必须是字符串列表")
    if any(not isinstance(v, str) or not normalize(v) for v in value):
        raise ValueError(f"{label} 包含空白或无效字符串")
    return tuple(normalize(v) for v in value)


@dataclass(frozen=True)
class Command:
    name: str
    phrases: tuple
    match: str
    kind: str
    target: str
    interface: str
    fields: dict
    cooldown: float


def parse_config(raw):
    if not isinstance(raw, dict):
        raise ValueError("配置文件顶层必须是 YAML mapping")
    unknown = set(raw) - {"audio", "matching", "commands"}
    if unknown:
        raise ValueError(f"未知配置项: {sorted(unknown)}")
    audio, matching = raw.get("audio", {}), raw.get("matching", {})
    if not isinstance(audio, dict) or not isinstance(matching, dict):
        raise ValueError("audio 和 matching 必须是 mapping")
    unknown = set(matching) - {"wake_words", "require_wake_word",
                               "max_command_age_sec", "service_timeout_sec"}
    if unknown:
        raise ValueError(f"未知 matching 配置项: {sorted(unknown)}")
    require_wake = matching.get("require_wake_word", True)
    if not isinstance(require_wake, bool):
        raise ValueError("require_wake_word 必须是布尔值")
    wake_words = string_list(matching.get("wake_words", ["小助手", "机器人"]),
                             "wake_words", allow_empty=not require_wake)
    max_age = number(matching.get("max_command_age_sec", 15.0),
                     "max_command_age_sec", 0.1)
    timeout = number(matching.get("service_timeout_sec", 8.0),
                     "service_timeout_sec", 0.1)
    rows = raw.get("commands")
    if not isinstance(rows, list) or not rows:
        raise ValueError("commands 必须是非空列表")
    commands, names = [], set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("每条 command 必须是 mapping")
        unknown = set(row) - {"name", "phrases", "match", "cooldown_sec", "action"}
        if unknown:
            raise ValueError(f"未知 command 配置项: {sorted(unknown)}")
        name = row.get("name")
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError(f"命令 name 为空或重复: {name!r}")
        names.add(name)
        phrases = string_list(row.get("phrases"), f"{name}.phrases")
        match = row.get("match", "exact")
        if match not in ("exact", "contains"):
            raise ValueError(f"{name}.match 只能是 exact 或 contains")
        action = row.get("action")
        if not isinstance(action, dict):
            raise ValueError(f"{name}.action 必须是 mapping")
        unknown = set(action) - {"kind", "name", "type", "fields"}
        if unknown:
            raise ValueError(f"未知 action 配置项: {sorted(unknown)}")
        kind, target, interface = (action.get(k) for k in ("kind", "name", "type"))
        if kind not in ("topic", "service"):
            raise ValueError(f"{name}.action.kind 只能是 topic 或 service")
        if not isinstance(target, str) or not target.strip():
            raise ValueError(f"{name}.action.name 不能为空")
        parts = interface.split("/") if isinstance(interface, str) else []
        expected = "msg" if kind == "topic" else "srv"
        if len(parts) != 3 or parts[1] != expected or not all(parts):
            raise ValueError(f"{name}.action.type 应为 package/{expected}/Type")
        fields = action.get("fields", {})
        if not isinstance(fields, dict) or not all(isinstance(k, str) for k in fields):
            raise ValueError(f"{name}.action.fields 必须是字符串键的 mapping")
        cooldown = number(row.get("cooldown_sec", 2.0), f"{name}.cooldown_sec")
        commands.append(Command(name, phrases, match, kind, target, interface, fields, cooldown))
    return audio, commands, wake_words, require_wake, max_age, timeout


def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return parse_config(yaml.safe_load(stream))


class CommandMatcher:
    def __init__(self, commands, wake_words, require_wake):
        self.commands = commands
        self.wake_words = sorted(set(wake_words), key=len, reverse=True)
        self.require_wake = require_wake
        self.last_dispatch = {}

    def match(self, text, now):
        text = normalize(text)
        if not text:
            return None, "empty"
        for word in self.wake_words:
            if text.startswith(word):
                text = text[len(word):]
                break
        else:
            if self.require_wake:
                return None, "wake_word_missing"
        matches = [c for c in self.commands if any(
            text == phrase if c.match == "exact" else phrase in text
            for phrase in c.phrases)]
        if not matches:
            return None, "no_match"
        if len(matches) > 1:
            return None, "ambiguous"
        command = matches[0]
        if now - self.last_dispatch.get(command.name, -math.inf) < command.cooldown:
            return None, "cooldown"
        return command, "matched"

    def mark_dispatched(self, command, now):
        self.last_dispatch[command.name] = now
