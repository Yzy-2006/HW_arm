import copy
from pathlib import Path
import shutil
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
from edit_plan import dataset_fingerprint, merge_plans, require_complete, validate_plan
from test_filter_lerobot import make_source
import edit_lerobot


def make_plan(fingerprint, episodes, ids, reviewer='A', reviewed=True, deletions=None):
    return {'version': 2, 'fingerprint': fingerprint,
            'episodes': [{'index': e['episode_index'], 'length': e['length']} for e in episodes],
            'assigned': ids, 'records': {str(i): {'reviewer': reviewer, 'reviewed': reviewed,
                                                'deletions': (deletions or {}).get(str(i), [])} for i in ids}}


def test_fingerprint_is_portable_and_checks_numeric_data_and_video_bytes(tmp_path):
    root, other = tmp_path / 'one', tmp_path / 'another-computer'
    make_source(root, videos=True)
    shutil.copytree(root, other)
    fingerprint = dataset_fingerprint(root)
    assert dataset_fingerprint(other) == fingerprint
    numeric = next((other / 'data').rglob('*.parquet'))
    original = numeric.read_bytes()
    numeric.write_bytes(original + b'changed')
    assert dataset_fingerprint(other) != fingerprint
    numeric.write_bytes(original)
    video = next((other / 'videos').rglob('*.mp4'))
    video.write_bytes(video.read_bytes() + b'changed')
    assert dataset_fingerprint(other) != fingerprint


def test_merge_tracks_unchanged_reviewed_episodes_and_requires_complete_dataset(tmp_path):
    _, episodes, _ = make_source(tmp_path / 'source')
    a = make_plan('sha256:test', episodes, [0])
    b = make_plan('sha256:test', episodes, [1], reviewer='B', deletions={'1': [[2, 3]]})
    with pytest.raises(ValueError, match='1'):
        require_complete(a, 'sha256:test', episodes)
    merged = merge_plans(a, b, 'sha256:test', episodes)['plan']
    assert merged['records']['0'] == {'deletions': [], 'reviewed': True, 'reviewer': 'A'}
    assert merged['records']['1']['deletions'] == [[2, 3]]
    assert require_complete(merged, 'sha256:test', episodes) == merged
    assert a['assigned'] == [0]


def test_conflicts_require_explicit_choice_even_for_matching_assignments(tmp_path):
    _, episodes, _ = make_source(tmp_path / 'source')
    a = make_plan('hash', episodes, [0])
    b = make_plan('hash', episodes, [0, 1], reviewer='B', reviewed=False)
    result = merge_plans(a, b, 'hash', episodes)
    assert result['plan'] is None
    assert result['conflicts'][0]['episode'] == 0
    kept = merge_plans(a, b, 'hash', episodes, {'0': 'current'})['plan']
    assert kept['records']['0']['reviewed']
    assert not kept['records']['1']['reviewed']
    replaced = merge_plans(a, b, 'hash', episodes, {'0': 'incoming'})['plan']
    assert replaced['records']['0']['reviewer'] == 'B'
    assert merge_plans(a, a, 'hash', episodes)['conflicts']
    with pytest.raises(ValueError):
        merge_plans(a, b, 'hash', episodes, {'0': 'union'})


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(version=1), lambda p: p.update(fingerprint='other'),
    lambda p: p['episodes'][0].update(length=99), lambda p: p.update(assigned=[0, 0]),
    lambda p: p.update(assigned=[-1]), lambda p: p['records']['0'].update(reviewed='yes'),
    lambda p: p['records']['0'].update(reviewer=''),
    lambda p: p['records']['0'].update(deletions=[[0, 100]]),
    lambda p: p['records'].update({'1': {'reviewer': 'B', 'reviewed': True, 'deletions': []}}),
])
def test_invalid_plans_are_rejected_without_modifying_current(tmp_path, mutation):
    _, episodes, _ = make_source(tmp_path / 'source')
    a = make_plan('hash', episodes, [0])
    before = copy.deepcopy(a)
    incoming = copy.deepcopy(a)
    mutation(incoming)
    with pytest.raises(ValueError):
        merge_plans(a, incoming, 'hash', episodes)
    assert a == before


def test_editor_export_requires_review_and_records_the_merged_plan(tmp_path):
    pytest.importorskip('lerobot.datasets.lerobot_dataset')
    root = tmp_path / 'source'
    _, episodes, _ = make_source(root, videos=True)
    app = edit_lerobot.Editor(root)
    try:
        app.pool.submit(lambda: None).result(timeout=30)
        fingerprint = app.fingerprint()
        a = make_plan(fingerprint, episodes, [0], deletions={'0': [[2, 3]]})
        b = make_plan(fingerprint, episodes, [1], reviewer='B')
        with pytest.raises(ValueError, match='1'):
            app.export({'dataset': str(root), 'output': str(tmp_path / 'incomplete'), 'plan': a})
        merged = app.import_plan({'current': a, 'incoming': b})['plan']
        out = tmp_path / 'output'
        app.export({'dataset': str(root), 'output': str(out), 'plan': merged})
        app.pool.submit(lambda: None).result(timeout=90)
        assert app.job['state'] == 'done', app.job
        import json
        report = json.loads((out / 'meta/manual_trim.json').read_text())
        assert report['review_plan'] == merged
        assert report['retained_frames'] == 10
        assert len(report['episodes']) == 3
        # A changed source cannot be exported using an earlier review identity.
        (root / 'meta/change.json').write_text('{}')
        app.export({'dataset': str(root), 'output': str(tmp_path / 'changed'), 'plan': merged})
        app.pool.submit(lambda: None).result(timeout=30)
        assert app.job['state'] == 'error'
        assert not (tmp_path / 'changed').exists()
    finally:
        app.pool.shutdown()
        app.close_readers()
