import json
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
from core import AsyncEpisodeWriter


def read_metadata(writer):
    return json.loads((writer.path / 'episode.json').read_text())


def test_ordered_snapshots_are_owned_and_finish_drains(tmp_path):
    entered, release = threading.Event(), threading.Event()
    pixels = []

    def write(path, image):
        entered.set()
        assert release.wait(2)
        pixels.append(int(image[0, 0, 0]))
        return True

    writer = AsyncEpisodeWriter(tmp_path, {}, write)
    image = np.zeros((2, 2, 3), np.uint8)
    sample = {'timestamp_s': 0., 'nested': {'q': [1.]}}
    try:
        writer.add(sample, {'head': image})
        assert entered.wait(1)
        sample['nested']['q'][0] = 2.
        sample['timestamp_s'] = .033
        image[:] = 5
        writer.add(sample, {'head': image})
        sample['nested']['q'][0] = 99.
        image[:] = 99
        release.set()
        path = writer.finish(2)
    finally:
        release.set()
        writer.abort('test cleanup')
        assert writer.done.wait(2)
    assert pixels == [0, 5]
    rows = [json.loads(row) for row in (path / 'frames.jsonl').read_text().splitlines()]
    assert [row['nested']['q'][0] for row in rows] == [1., 2.]
    assert [row['timestamp_s'] for row in rows] == [0., .033]
    assert json.loads((path / 'episode.json').read_text())['frames'] == 2


@pytest.mark.parametrize('failure', ['overflow', 'timeout', 'abort', 'disk'])
def test_failure_never_completes_and_pending_frames_are_cancelled(tmp_path, failure):
    entered, release = threading.Event(), threading.Event()

    def write(*args):
        entered.set()
        assert release.wait(2)
        if failure == 'disk':
            raise OSError('disk full')
        return True

    writer = AsyncEpisodeWriter(tmp_path, {}, write, capacity=1, finish_timeout=.05)
    images = {'head': np.zeros((2, 2, 3), np.uint8)}
    try:
        writer.add({'timestamp_s': 0.}, images)
        assert entered.wait(1)
        writer.add({'timestamp_s': .033}, images)
        if failure == 'overflow':
            with pytest.raises(RuntimeError, match='queue full'):
                writer.add({'timestamp_s': .066}, images)
        elif failure == 'timeout':
            with pytest.raises(TimeoutError, match='drain timed out'):
                writer.finish()
        elif failure == 'abort':
            writer.abort('operator shutdown')
        release.set()
        assert writer.done.wait(2)
        with pytest.raises(RuntimeError):
            writer.finish()
        meta = read_metadata(writer)
        assert meta['status'] == 'aborted' and meta['frames'] <= 1
        assert list(tmp_path.iterdir()) == [writer.path]
        assert writer.path.name.endswith('.partial')
        assert writer.stats()['queued'] == 0
        if failure == 'disk':
            assert 'disk full' in meta['reason']
    finally:
        release.set()
        writer.abort('test cleanup')
        assert writer.done.wait(2)


def test_discard_waits_for_inflight_write_before_removing_directory(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def write(*args):
        entered.set()
        assert release.wait(2)
        return True
    writer = AsyncEpisodeWriter(tmp_path, {}, write)
    writer.add({}, {'head': np.zeros((1, 1, 3), np.uint8)})
    assert entered.wait(1)
    result = []
    discarding = threading.Thread(target=lambda: result.append(writer.discard()))
    try:
        discarding.start()
        discarding.join(.05)
        assert discarding.is_alive() and writer.path.exists()
    finally:
        release.set()
        discarding.join(2)
    assert result == [None] and not writer.path.exists()
    assert writer.done.is_set()


def test_too_short_stays_partial_and_nonfinite_samples_are_rejected(tmp_path):
    writer = AsyncEpisodeWriter(tmp_path, {}, lambda *args: True)
    writer.add({'timestamp_s': 0.}, {})
    assert writer.finish(2) is None
    assert read_metadata(writer)['status'] == 'too_short'
    writer = AsyncEpisodeWriter(tmp_path, {}, lambda *args: True)
    writer.add({'timestamp_s': float('nan')}, {})
    assert writer.done.wait(2)
    assert read_metadata(writer)['status'] == 'aborted'


def test_owned_read_only_camera_images_are_retained_without_another_copy(tmp_path):
    class CameraImage(np.ndarray):
        def copy(self, *args, **kwargs):
            pytest.fail("An immutable camera exposure must not be copied again during capture")

    image = np.ndarray.__new__(CameraImage, shape=(2, 2, 3), dtype=np.uint8)
    image.fill(7)
    image.setflags(write=False)
    entered, release = threading.Event(), threading.Event()
    written = []

    def write(path, pixels):
        entered.set()
        assert release.wait(2)
        written.append(pixels)
        return True

    writer = AsyncEpisodeWriter(tmp_path, {}, write)
    images = {'head': image}
    sample = {'timestamp_s': 0., 'nested': {'q': [1.]}}
    try:
        writer.add(sample, images, copy_images=False)
        assert entered.wait(1)
        images['head'] = np.zeros((2, 2, 3), np.uint8)
        sample['nested']['q'][0] = 9.
        with pytest.raises(ValueError):
            image[:] = 3
        release.set()
        path = writer.finish()
        assert written[0] is image and np.all(written[0] == 7)
        row = json.loads((path / 'frames.jsonl').read_text().strip())
        assert row['nested']['q'] == [1.]
        assert set(writer.stats()['enqueue_times']) == {
            'check_lock', 'sample_copy', 'image_prepare', 'commit_lock'}
    finally:
        release.set()
        writer.abort('test cleanup')
        assert writer.done.wait(2)


@pytest.mark.parametrize('kind', ['mutable', 'aliased_view'])
def test_shared_images_reject_mutable_or_aliased_storage(tmp_path, kind):
    owner = np.zeros((2, 2, 3), np.uint8)
    image = owner if kind == 'mutable' else owner.view()
    if kind == 'aliased_view':
        image.setflags(write=False)
    writer = AsyncEpisodeWriter(tmp_path, {}, lambda *args: True)
    try:
        with pytest.raises(ValueError, match='own their storage and be read-only'):
            writer.add({}, {'head': image}, copy_images=False)
        assert writer.frames == 0
    finally:
        writer.abort('test cleanup')
        assert writer.done.wait(2)


def test_slow_image_preparation_does_not_hold_writer_lock_and_abort_wins(tmp_path):
    entered, release, accessed = threading.Event(), threading.Event(), threading.Event()
    errors = []

    class SlowImage:
        def copy(self):
            entered.set()
            assert release.wait(2)
            return np.zeros((2, 2, 3), np.uint8)

    writer = AsyncEpisodeWriter(tmp_path, {}, lambda *args: True)

    def add():
        try:
            writer.add({}, {'head': SlowImage()})
        except RuntimeError as exc:
            errors.append(str(exc))

    def abort():
        writer.stats()
        writer.abort('cancel during image preparation')
        accessed.set()

    adding, aborting = threading.Thread(target=add), threading.Thread(target=abort)
    try:
        adding.start()
        assert entered.wait(1)
        aborting.start()
        assert accessed.wait(1), 'Image copying must not block writer stats or cancellation'
    finally:
        release.set()
        adding.join(2)
        if aborting.ident is not None:
            aborting.join(2)
        writer.abort('test cleanup')
        assert writer.done.wait(2)
    assert errors == ['cancel during image preparation']
    assert writer.frames == 0 and read_metadata(writer)['status'] == 'aborted'
