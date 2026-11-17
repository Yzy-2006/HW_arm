import json
from io import BytesIO
from pathlib import Path
import sys
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "collection"))
import edit_lerobot as editor
import filter_lerobot as filtering
from test_filter_lerobot import make_source, fingerprints


def test_ranges_are_inclusive_and_merged():
    assert editor.normalize_ranges([[4, 4], [1, 2], [2, 3]], 6) == [[1, 4]]


@pytest.mark.parametrize("ranges", [[[0, 6]], [[-1, 0]], [[3, 2]], [[True, 2]], [[0.5, 2]], [[2]], None])
def test_bad_ranges_are_rejected(ranges):
    with pytest.raises(ValueError):
        editor.normalize_ranges(ranges, 6)


def test_plan_splits_gaps_preserves_unedited_episodes_and_drops_deleted_ones(tmp_path):
    root = tmp_path / "source"
    make_source(root)
    _, sources, _, report = editor.plan_trim(root, {"0": [[2, 3]]})
    assert [e['episode_index'] for e in sources] == [0, 0, 1]
    assert [e['source_frame_indices'] for e in report['episodes']] == [[0, 1], [4, 5], list(range(6))]
    assert report['retained_frames'] == 10
    _, sources, _, report = editor.plan_trim(root, {"0": [[0, 0], [5, 5]], "1": [[0, 5]]})
    assert len(sources) == 1
    assert report['episodes'][0]['source_frame_indices'] == [1, 2, 3, 4]
    with pytest.raises(ValueError, match="every frame"):
        editor.plan_trim(root, {"0": [[0, 5]], "1": [[0, 5]]})
    with pytest.raises(ValueError, match="unknown episodes"):
        editor.plan_trim(root, {"20": []})


def test_export_rebuilds_video_alignment_and_provenance_without_touching_source(tmp_path):
    pytest.importorskip('lerobot.datasets.lerobot_dataset')
    root, output = tmp_path / 'source', tmp_path / 'trimmed'
    make_source(root, videos=True)
    before = fingerprints(root)
    info, episodes, tasks, report = editor.plan_trim(root, {"0": [[2, 3]], "1": [[0, 5]]})
    filtering.write_filtered(root, output, info, episodes, tasks, report)
    out_info, out_episodes, _ = filtering.load_source(output)
    assert out_info['total_frames'] == 4
    assert out_info['total_episodes'] == 2
    assert fingerprints(root) == before
    assert not (output / 'FILTER_INCOMPLETE').exists()
    for episode, expected in zip(out_episodes, ([0, 1], [4, 5])):
        rows = filtering.read_episode(output, out_info, episode)
        np.testing.assert_allclose([r['action'][0] for r in rows], np.array(expected) * .1)
        for camera_index, camera in enumerate(['observation.images.head', 'observation.images.left_wrist']):
            timestamps = [episode[f'videos/{camera}/from_timestamp'] + r['timestamp'] for r in rows]
            frames = list(filtering.selected_video_frames(filtering.video_path(output, out_info, episode, camera), timestamps, 30))
            np.testing.assert_allclose([frame.mean() for frame in frames],
                                       30 + np.array(expected) * 12 + camera_index * 20, atol=8)
    assert json.loads((output / 'meta/manual_trim.json').read_text()) == report
    assert json.loads((output / 'meta/stats.json').read_text())['action']['count'] == [4]
    schema = json.loads((output / 'meta/towel_schema.json').read_text())
    assert schema['manual_trim']['provenance'] == 'meta/manual_trim.json'


def test_http_preview_authentication_and_invalid_requests(tmp_path):
    root = tmp_path / 'source'
    make_source(root, videos=True)
    app = editor.Editor(root)
    for frame in [0, 1, 2, 5, 3, 4, 0]:
        image = np.asarray(Image.open(BytesIO(app.preview(0, frame, 'observation.images.head'))))
        assert abs(image.mean() - (30 + frame * 12)) < 8
    server = ThreadingHTTPServer(('127.0.0.1', 0), editor.make_handler(app, 'test-token'))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    def get(path, token='test-token'):
        return urlopen(Request(url + path, headers={'X-Editor-Token': token}), timeout=10)
    try:
        assert json.load(get('/api/dataset'))['total_frames'] == 12
        image = get('/api/frame?episode=1&frame=3&camera=observation.images.head').read()
        assert image.startswith(b'\xff\xd8')
        for path, token, status in [('/api/status', 'wrong', 403),
                                    ('/api/frame?episode=-1&frame=0&camera=x', 'test-token', 400),
                                    ('/api/frame?episode=0&frame=6&camera=observation.images.head', 'test-token', 400)]:
            with pytest.raises(HTTPError) as exc:
                get(path, token)
            assert exc.value.code == status
        with pytest.raises(ValueError):
            app.export({'dataset': str(root), 'output': str(root), 'deletions': {}})
        app.job = {'state': 'running'}
        with pytest.raises(ValueError, match='export'):
            app.load(root)
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
        app.pool.shutdown()
        app.close_readers()
