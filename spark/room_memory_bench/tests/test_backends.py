import json
import os
import sys
import pytest
from spark.room_memory_bench.backends import BackendUnavailable, SubprocessBackend, validate_detection
from spark.room_memory_bench.generator import ReplayConfig, replay
from spark.room_memory_bench.runner import current_rss, main


# Protocol fixture, not a C++ implementation: verifies framing, payloads and lifecycle.
SERVER = '''
import json, struct, sys, time
mode = MODE
source = sys.stdin.buffer
while True:
    prefix = source.read(4)
    if not prefix:
        break
    header = json.loads(source.read(struct.unpack('<I', prefix)[0]))
    op = header['op']
    if mode == 'hang':
        time.sleep(30)
    if mode == 'invalid':
        print('not json', flush=True)
        continue
    if op == 'init':
        assert header['config']['state_dir'] is None
        reply = {'op': 'ready', 'version': 1}
    elif op == 'tick':
        for frame in header['frames']:
            data = source.read(frame['bytes'])
            assert len(data) == frame['points'] * 12
            assert all(abs(x) < 20 for x in struct.unpack('<' + 'f' * (len(data)//4), data))
        reply = {'op': 'events', 'index': header['index'], 'events': []}
    else:
        print(json.dumps({'op': 'bye'}), flush=True)
        break
    print(json.dumps(reply), flush=True)
'''


def executable(tmp_path, mode='valid'):
    script = tmp_path / 'backend'
    script.write_text('#!' + sys.executable + '\n' + SERVER.replace('MODE', repr(mode)))
    script.chmod(0o755)
    return script


def test_stdio_roundtrip(tmp_path):
    backend = SubprocessBackend(executable(tmp_path), tmp_path / 'stderr', timeout=5)
    try:
        assert current_rss(backend.pid) > 0
        for tick in replay(ReplayConfig(duration=1, points=32)):
            assert backend.tick(tick) == []
    finally:
        backend.close()
    assert backend.process.returncode == 0


@pytest.mark.parametrize('mode,exception', [('invalid', ValueError), ('hang', TimeoutError)])
def test_bad_or_stalled_backend_fails_and_is_cleaned_up(tmp_path, mode, exception):
    with pytest.raises(exception):
        SubprocessBackend(executable(tmp_path, mode), tmp_path / 'stderr', timeout=.3 if mode == 'hang' else 5)


def test_missing_backend_is_a_reported_skip(tmp_path):
    with pytest.raises(BackendUnavailable):
        SubprocessBackend(tmp_path / 'absent', tmp_path / 'stderr')
    (tmp_path / 'ticks.jsonl').write_text('stale output from a previous run')
    assert main(['--backend', 'cpp', '--binary', str(tmp_path / 'absent'), '--output', str(tmp_path), '--duration', '1']) == 0
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['status'] == 'skipped' and 'missing' in report['reason']
    assert (tmp_path / 'report.md').exists()
    assert not (tmp_path / 'ticks.jsonl').exists()


def test_detection_validation():
    event = {'sensor': 'jt128', 'type': 'appeared', 'state': 'candidate', 'event_id': 1,
             'time_s': 1., 'bbox': [[0, 0, 0], [1, 1, 1]]}
    validate_detection(event, 1.)
    for changes in ({'time_s': 0.}, {'bbox': [[0, 0, 0], [-1, 1, 1]]}, {'sensor': 'unknown'},
                    {'bbox': [[float('nan'), 0, 0], [1, 1, 1]]}):
        with pytest.raises(ValueError):
            validate_detection({**event, **changes}, 1.)


def test_current_rss_and_short_python_cli(tmp_path):
    assert current_rss(os.getpid()) > 0
    assert main(['--duration', '1', '--points', '32', '--output', str(tmp_path)]) == 0
    report = json.loads((tmp_path / 'report.json').read_text())
    assert report['status'] == 'complete' and report['ticks'] == 10
    assert len((tmp_path / 'ticks.jsonl').read_text().splitlines()) == 10
    assert report['harness_rss']['start_bytes'] > 0
