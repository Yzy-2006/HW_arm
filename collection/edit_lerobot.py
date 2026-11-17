#!/usr/bin/env python3
"""Local visual editor for exported towel LeRobot v3 datasets."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
from pathlib import Path
import secrets
import threading
from urllib.parse import parse_qs, urlparse

from PIL import Image
from edit_plan import dataset_fingerprint, merge_plans, normalize_ranges, require_complete, validate_plan

from filter_lerobot import load_source, read_episode, selected_video_frames, validate_output, video_path, write_filtered


def plan_trim(root, deletions):
    info, episodes, tasks = load_source(root)
    if not isinstance(deletions, dict) or set(deletions) - {str(e['episode_index']) for e in episodes}:
        raise ValueError("Deletion plan references unknown episodes")
    report = {
        "operation": "manual_trim", "source_dataset": str(root), "fps": info["fps"],
        "timeline": "each contiguous retained range becomes an episode on the original fixed FPS grid",
        "deletions": {}, "episodes": [], "source_frames": info["total_frames"],
    }
    sources = []
    for episode in episodes:
        key = str(episode["episode_index"])
        ranges = normalize_ranges(deletions.get(key, []), episode["length"])
        report["deletions"][key] = ranges
        rows = read_episode(root, info, episode)
        if any(row["task_index"] not in tasks for row in rows):
            raise ValueError("Frame references an unknown task")
        for camera, spec in info["features"].items():
            if spec["dtype"] == "video" and not video_path(root, info, episode, camera).is_file():
                raise ValueError(f"Missing video: episode {key}, {camera}")
        start = 0
        for first, last in ranges + [[len(rows), len(rows)]]:
            if start < first:
                kept = list(range(start, first))
                sources.append(episode)
                report["episodes"].append({
                    "source_episode_index": episode["episode_index"],
                    "output_episode_index": len(sources) - 1, "source_frames": len(rows),
                    "retained_frames": len(kept), "source_frame_indices": kept,
                    "source_timestamps_s": [rows[i]["timestamp"] for i in kept],
                })
            start = last + 1
    report["retained_frames"] = sum(e["retained_frames"] for e in report["episodes"])
    if not sources:
        raise ValueError("The plan deletes every frame in the dataset")
    return info, sources, tasks, report


class Editor:
    def __init__(self, root=None):
        self.lock = threading.RLock()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.job = {"state": "idle"}
        self.root = None
        self.readers = {}
        self.identity = {"state": "idle"}
        self.generation = 0
        if root:
            self.load(root)

    def load(self, path):
        with self.lock:
            if self.job["state"] == "running":
                raise ValueError("Wait for the current export to finish")
            root = Path(path).expanduser().resolve()
            info, episodes, tasks = load_source(root)
            cameras = [key for key, spec in info["features"].items() if spec["dtype"] == "video"]
            if not cameras:
                raise ValueError("This dataset has no video cameras")
            self.close_readers()
            self.root, self.info, self.episodes, self.tasks = root, info, episodes, tasks
            self.cameras = cameras
            self.generation += 1
            generation = self.generation
            self.identity = {"state": "running", "percent": 0}
            def progress(done, total):
                with self.lock:
                    if generation == self.generation:
                        self.identity["percent"] = round(100 * done / max(total, 1))
            def identify():
                try:
                    result = {"state": "done", "fingerprint": dataset_fingerprint(root, progress)}
                except Exception as exc:
                    result = {"state": "error", "message": str(exc)}
                with self.lock:
                    if generation == self.generation:
                        self.identity = result
            self.pool.submit(identify)
            self.preview.cache_clear()
            return self.describe()

    def close_readers(self):
        for _, _, reader in self.readers.values():
            reader.close()
        self.readers.clear()

    def describe(self):
        if self.root is None:
            return None
        return {"path": str(self.root), "fps": self.info["fps"], "cameras": self.cameras,
                "identity": self.identity.copy(),
                "total_frames": self.info["total_frames"],
                "episodes": [{"index": e["episode_index"], "length": e["length"]} for e in self.episodes]}

    def fingerprint(self):
        if self.identity["state"] != "done":
            raise ValueError("请等待数据集内容校验完成")
        return self.identity["fingerprint"]

    def import_plan(self, body):
        with self.lock:
            fingerprint = self.fingerprint()
            if body.get("mode") == "replace":
                return {"plan": validate_plan(body["incoming"], fingerprint, self.episodes), "conflicts": []}
            return merge_plans(body["current"], body["incoming"], fingerprint, self.episodes,
                               body.get("choices"))

    @lru_cache(maxsize=48)
    def preview(self, episode_index, frame_index, camera):
        episode = self.episodes[episode_index]
        if camera not in self.cameras or not 0 <= frame_index < episode["length"]:
            raise ValueError("Invalid camera or frame")
        previous_episode, previous_frame, reader = self.readers.get(camera, (None, None, None))
        if previous_episode != episode_index or previous_frame != frame_index - 1:
            if reader is not None:
                reader.close()
            offset = episode[f"videos/{camera}/from_timestamp"]
            timestamps = [offset + i / self.info["fps"] for i in range(frame_index, episode["length"])]
            reader = selected_video_frames(video_path(self.root, self.info, episode, camera), timestamps, self.info["fps"])
        self.readers[camera] = (episode_index, frame_index, reader)
        image = Image.fromarray(next(reader))
        image.thumbnail((800, 600))
        output = BytesIO()
        image.save(output, "JPEG", quality=85)
        return output.getvalue()

    def export(self, body):
        with self.lock:
            if self.root is None or body.get("dataset") != str(self.root):
                raise ValueError("Reload the dataset before exporting")
            if self.job["state"] == "running":
                raise ValueError("An export is already running")
            if not isinstance(body.get("output"), str) or not body["output"].strip():
                raise ValueError("Choose a new output directory")
            output = Path(body["output"]).expanduser().resolve()
            validate_output(self.root, output)
            review_plan = require_complete(body.get("plan"), self.fingerprint(), self.episodes)
            # Validate in the worker so large datasets do not block preview/status requests.
            root = self.root
            deletions = {key: record["deletions"] for key, record in review_plan["records"].items()}
            self.job = {"state": "running", "output": str(output)}
            def run():
                try:
                    if dataset_fingerprint(root) != review_plan["fingerprint"]:
                        raise ValueError("源数据已发生变化，请重新打开数据集并核对方案")
                    info, episodes, tasks, report = plan_trim(root, deletions)
                    report["review_plan"] = review_plan
                    write_filtered(root, output, info, episodes, tasks, report)
                    job = {"state": "done", "output": str(output),
                           "frames": report["retained_frames"], "episodes": len(episodes)}
                except Exception as exc:
                    job = {"state": "error", "message": str(exc), "output": str(output)}
                with self.lock:
                    self.job = job
            self.pool.submit(run)
            return self.job.copy()


def make_handler(editor, token):
    class Handler(BaseHTTPRequestHandler):
        def valid_host(self):
            return self.headers.get("Host") in {
                f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}

        def send(self, data, content_type="application/json", status=200):
            if content_type == "application/json":
                data = json.dumps(data).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            try:
                if not self.valid_host():
                    return self.send({"error": "Invalid host"}, status=403)
                url = urlparse(self.path)
                if url.path == "/":
                    page = Path(__file__).with_name("editor.html").read_text().replace("__TOKEN__", token)
                    return self.send(page.encode(), "text/html; charset=utf-8")
                if self.headers.get("X-Editor-Token") != token:
                    return self.send({"error": "Invalid session"}, status=403)
                with editor.lock:
                    if url.path == "/api/dataset":
                        return self.send(editor.describe())
                    if url.path == "/api/status":
                        return self.send(editor.job)
                    if url.path == "/api/frame" and editor.root:
                        query = parse_qs(url.query)
                        episode, frame = int(query["episode"][0]), int(query["frame"][0])
                        if not 0 <= episode < len(editor.episodes):
                            raise ValueError("Invalid episode")
                        return self.send(editor.preview(episode, frame, query["camera"][0]), "image/jpeg")
                self.send({"error": "Not found"}, status=404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (ValueError, KeyError, IndexError, OSError) as exc:
                self.send({"error": str(exc)}, status=400)
            except Exception as exc:
                self.send({"error": str(exc)}, status=500)

        def do_POST(self):
            if not self.valid_host() or self.headers.get("X-Editor-Token") != token:
                return self.send({"error": "Invalid session"}, status=403)
            try:
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 20_000_000:
                    raise ValueError("Invalid request size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("Expected a JSON object")
                if self.path == "/api/load":
                    return self.send(editor.load(body["path"]))
                if self.path == "/api/export":
                    return self.send(editor.export(body))
                if self.path == "/api/plan":
                    return self.send(editor.import_plan(body))
                self.send({"error": "Not found"}, status=404)
            except Exception as exc:
                self.send({"error": str(exc)}, status=400)

        def log_message(self, *_args):
            pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    editor = Editor(args.dataset)
    for port in range(args.port, args.port + 20):
        try:
            server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(editor, secrets.token_urlsafe(32)))
            break
        except OSError as exc:
            if exc.errno != 98 or port == args.port + 19:
                raise
    print(f"Dataset editor: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        editor.pool.shutdown(wait=True)
        editor.close_readers()


if __name__ == "__main__":
    main()
