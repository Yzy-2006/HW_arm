"""Managed camera/recorder process with bounded latest-image shared memory."""

import json
import logging
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np


class SharedJson:
    def __init__(self, context, capacity=65536):
        self.buffer = context.RawArray('B', capacity)
        self.length = context.RawValue('i', 0)
        self.lock = context.Lock()

    def write(self, value):
        payload = json.dumps(value, allow_nan=False).encode('utf-8')
        if len(payload) > len(self.buffer):
            raise ValueError('Shared metadata capacity exceeded')
        if not self.lock.acquire(timeout=.1):
            raise TimeoutError('Recorder status lock timed out')
        try:
            self.buffer[:len(payload)] = payload
            self.length.value = len(payload)
        finally:
            self.lock.release()

    def read(self):
        if not self.lock.acquire(timeout=.1):
            raise TimeoutError('Recorder status lock timed out')
        try:
            payload = bytes(self.buffer[:self.length.value])
        finally:
            self.lock.release()
        return json.loads(payload) if payload else {}


class SharedFrames:
    def __init__(self, context, config):
        self.names = tuple(config['cameras'])
        self.shape = (int(config['height']), int(config['width']), 3)
        if not self.names or min(self.shape) <= 0:
            raise ValueError('Invalid shared camera layout')
        self.frame_size = int(np.prod(self.shape))
        self.pixels = context.RawArray('B', self.frame_size * len(self.names))
        self.metadata = SharedJson(context)

    def publish(self, images, metadata):
        if set(images) != set(self.names) or set(metadata) != set(self.names):
            raise ValueError('Incomplete shared camera snapshot')
        for image in images.values():
            if image.shape != self.shape or image.dtype != np.uint8:
                raise ValueError('Unexpected shared camera image layout')
        payload = json.dumps({'cameras': metadata}, allow_nan=False).encode('utf-8')
        if len(payload) > len(self.metadata.buffer):
            raise ValueError('Camera metadata capacity exceeded')
        if not self.metadata.lock.acquire(timeout=.1):
            raise TimeoutError('Shared camera lock timed out')
        try:
            pixels = np.frombuffer(self.pixels, dtype=np.uint8).reshape(len(self.names), *self.shape)
            for i, name in enumerate(self.names):
                pixels[i] = images[name]
            self.metadata.buffer[:len(payload)] = payload
            self.metadata.length.value = len(payload)
        finally:
            self.metadata.lock.release()

    def snapshot(self):
        if not self.metadata.lock.acquire(timeout=.1):
            raise TimeoutError('Shared camera lock timed out')
        try:
            payload = bytes(self.metadata.buffer[:self.metadata.length.value])
            info = json.loads(payload) if payload else {}
            if info.get('error'):
                raise ValueError(info['error'])
            if 'cameras' not in info:
                raise ValueError('Waiting for all cameras')
            pixels = np.frombuffer(self.pixels, dtype=np.uint8).reshape(len(self.names), *self.shape)
            images = {name: pixels[i].copy() for i, name in enumerate(self.names)}
            return images, info['cameras']
        finally:
            self.metadata.lock.release()


class RemoteCameras:
    def __init__(self, owner, config):
        self.owner, self.config = owner, config

    def snapshot(self, now):
        self.owner.check_health()
        images, metadata = self.owner.frames.snapshot()
        stamps = [meta['receive_monotonic_s'] for meta in metadata.values()]
        if self.config.get('camera_timing_checks_enabled', True):
            if any(now - stamp > self.config['max_camera_age_s'] for stamp in stamps):
                raise ValueError('Camera frame is stale')
            if max(stamps) - min(stamps) > self.config['max_camera_skew_s']:
                raise ValueError('Camera receive-time skew exceeds limit')
        return images, metadata

    def close(self):
        pass  # Camera lifetime belongs to the managed recorder process.


def _run_recorder(config, frames, status, heartbeat, ready, stop, connection, camera_factory=None):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
    import cv2
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from record import Cameras, make_node
    from ros_shutdown import drain_executor

    def request_stop(*_):
        stop.value = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    cv2.setNumThreads(1)
    cameras = node = executor = relay = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        cameras = (camera_factory or Cameras)(config)
        node = make_node(config, cameras, status_callback=status.write)
        node.publish_status()

        def relay_images():
            previous = None
            while not stop.value:
                heartbeat.value = time.monotonic()
                try:
                    images, metadata = cameras.snapshot(time.monotonic())
                    ids = tuple(metadata[name]['frame_number'] for name in frames.names)
                    if ids != previous:
                        frames.publish(images, metadata)
                        previous = ids
                except Exception as exc:
                    previous = None
                    try:
                        frames.metadata.write({'error': str(exc)})
                    except Exception:
                        stop.value = True
                time.sleep(.005)

        def commands():
            try:
                if connection.poll():
                    command = connection.recv()
                    response = node.record(SimpleNamespace(data=command['record']), SimpleNamespace())
                    connection.send(dict(id=command['id'], success=response.success, message=response.message,
                                         session=node.recording_status()['session']))
            except (EOFError, BrokenPipeError, OSError):
                stop.value = True

        node.create_timer(.01, commands)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        relay = threading.Thread(target=relay_images, name='camera-share', daemon=True)
        relay.start()
        heartbeat.value = time.monotonic()
        ready.set()
        node.get_logger().info(f'Independent recorder process ready: pid={os.getpid()}')
        parent = mp.parent_process()
        while rclpy.ok() and not stop.value and (parent is None or parent.is_alive()):
            executor.spin_once(timeout_sec=.05)
    except Exception as exc:
        status.write({'fatal': str(exc)})
        ready.set()
        raise
    finally:
        stop.value = True
        if executor is not None:
            drain_executor(executor)
        if node is not None:
            node.close()
        if relay is not None:
            relay.join(timeout=2.)
        if cameras is not None:
            cameras.close()
        connection.close()
        if rclpy.ok():
            rclpy.shutdown()


class RecorderProcess:
    def __init__(self, config, *, camera_factory=None, startup_timeout=30.):
        context = mp.get_context('spawn')
        self.frames = SharedFrames(context, config)
        self.status = SharedJson(context)
        self.heartbeat = context.RawValue('d', time.monotonic())
        # A killed child can leave an Event's internal lock acquired.
        self.stop, ready = context.RawValue('b', False), context.Event()
        self.connection, child_connection = context.Pipe()
        self.writer_finish_timeout_s = float(config.get('writer_finish_timeout_s', 10.))
        self.startup_timeout_s = float(config.get('startup_timeout_s', 3.))
        self.cameras = RemoteCameras(self, config)
        self._lock = threading.Lock()
        self._pending = {}
        self._next_id = 0
        self._expected_session = None
        self._reader = None
        self._closed = False
        self._monitor = None
        self._monitor_stop = context.RawValue('b', False)
        self.diagnostics_path = (Path(__file__).resolve().parents[1] / 'outputs/runtime_diagnostics' /
                                 f'intervention_{time.time_ns()}_{os.getpid()}.jsonl')
        config = dict(config)
        config['episode_metadata'] = {**config.get('episode_metadata', {}),
                                      'runtime_diagnostics': str(self.diagnostics_path)}
        self.process = context.Process(target=_run_recorder,
            args=(config, self.frames, self.status, self.heartbeat, ready, self.stop,
                  child_connection, camera_factory), name='towel-recorder')
        self.process.start()
        child_connection.close()
        try:
            deadline = time.monotonic() + startup_timeout
            while not ready.wait(.05):
                if not self.process.is_alive():
                    raise RuntimeError(f'Recorder process exited during startup: {self.status.read()}')
                if time.monotonic() > deadline:
                    raise TimeoutError('Recorder process startup timed out')
            self.check_health()
            self._reader = threading.Thread(target=self._receive, name='recorder-replies', daemon=True)
            self._reader.start()
            from runtime_diagnostics import monitor_resources
            self._monitor = context.Process(target=monitor_resources,
                args=(os.getpid(), self.process.pid, str(self.diagnostics_path), self.status, self._monitor_stop),
                name='towel-diagnostics')
            self._monitor.start()
            logging.getLogger(__name__).info('Runtime diagnostics: %s (bridge pid=%s, recorder pid=%s)',
                                              self.diagnostics_path, os.getpid(), self.process.pid)
        except Exception:
            self.close()
            raise

    def check_health(self):
        if not self.process.is_alive():
            raise RuntimeError(f'Recorder process exited (code={self.process.exitcode})')
        state = self.status.read()
        if state.get('fatal'):
            raise RuntimeError(f"Recorder process failed: {state['fatal']}")
        if time.monotonic() - self.heartbeat.value > 5.:
            raise RuntimeError('Recorder camera relay has not responded for 5 seconds')
        return state

    def ready_for_capture(self):
        state = self.check_health()
        with self._lock:
            expected = self._expected_session
        if expected is None or state.get('session') != expected:
            return False
        if not state.get('active'):
            raise RuntimeError(state.get('error') or 'Correction recording stopped unexpectedly')
        return bool(state.get('ready'))

    def request(self, record):
        self.check_health()
        future = Future()
        with self._lock:
            request_id = self._next_id
            self._next_id += 1
            self._pending[request_id] = (future, record)
            try:
                self.connection.send({'id': request_id, 'record': bool(record)})
            except Exception:
                self._pending.pop(request_id)
                raise
        return future

    def _receive(self):
        error = RuntimeError('Recorder process stopped')
        try:
            while not self._closed:
                if self.connection.poll(.05):
                    result = self.connection.recv()
                    with self._lock:
                        future, recording = self._pending.pop(result['id'])
                        if result['success']:
                            self._expected_session = result['session'] if recording else None
                    future.set_result(SimpleNamespace(success=result['success'], message=result['message']))
                elif not self.process.is_alive():
                    break
        except Exception as exc:
            error = RuntimeError(f'Recorder connection failed: {exc}')
        finally:
            with self._lock:
                pending, self._pending = self._pending, {}
            for future, _ in pending.values():
                if not future.done():
                    future.set_exception(error)

    def close(self):
        if self._closed:
            return
        self.stop.value = True
        self.process.join(timeout=self.writer_finish_timeout_s + 5.)
        if self.process.is_alive():
            logging.getLogger(__name__).warning('Recorder shutdown timed out; terminating, .partial retained')
            self.process.terminate()
            self.process.join(timeout=2.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=2.)
        self._closed = True
        if self._reader is not None:
            self._reader.join(timeout=1.)
        self.connection.close()
        self._monitor_stop.value = True
        if self._monitor is not None and self._monitor.pid is not None:
            self._monitor.join(timeout=3.)
            if self._monitor.is_alive():
                self._monitor.kill()
                self._monitor.join(timeout=2.)
            if self._monitor.exitcode:
                logging.getLogger(__name__).warning('Resource diagnostics exited with code %s', self._monitor.exitcode)
