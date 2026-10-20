"""Hosted gate checks. Standard library only; reports never replace test exit codes."""
import argparse
from collections import Counter
from html import escape
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


DATASET_TESTS = {'static_longrun', 'static_longrun_b', 'static_recording'}
FIXTURE_GROUPS = (
    'Tests/Fixtures/room/golden/hand',
    'Tests/Fixtures/room/snapshots',
    'Tests/Fixtures/room/synthetic',
    'Tests/Fixtures/change-detection',
    'spark/room_memory/tests/fixtures/hand',
    'spark/room_memory/tests/fixtures/swift',
    'spark/room_memory_cpp/tests/fixtures/static5m-mid360-excerpt.mcap',
)


def compare_goldens(left, right):
    contents = [{p.name: p.read_bytes() for p in root.glob('*.hex')} for root in (left, right)]
    a, b = contents
    if not a or not b:
        raise ValueError('hand golden sets must both be nonempty')
    if a.keys() != b.keys():
        raise ValueError(f'hand golden names differ: {sorted(a.keys() ^ b.keys())}')
    bad = [name for name in sorted(a) if not a[name] or not b[name] or a[name] != b[name]]
    if bad:
        raise ValueError(f'empty or byte-different hand goldens: {bad}')
    return len(a)


def check_fixtures(root):
    # The git index supplies the complete committed inventory, including files
    # deleted from the worktree; an empty glob must not silently reduce coverage.
    for group in FIXTURE_GROUPS:
        names = subprocess.check_output(
            ['git', 'ls-files', '-z', '--', group], cwd=root).decode().split('\0')
        names = [name for name in names if name]
        if not names:
            raise ValueError(f'no committed fixtures in {group}')
        for name in names:
            path = root / name
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f'missing or empty committed fixture: {name}')
    count = compare_goldens(root / FIXTURE_GROUPS[0], root / FIXTURE_GROUPS[4])
    print(f'Committed fixtures present; {count} hand goldens identical byte-for-byte')


def summarize_report(path, kind, require_rosbags=False):
    title = 'CTest entries' if kind == 'ctest' else 'pytest cases'
    try:
        root = ET.parse(path).getroot()
        cases = list(root.iter('testcase'))
        # pytest has a testsuites wrapper; CTest writes a single testsuite.
        declared = root.get('tests')
        if declared is None:
            declared = sum(int(s.get('tests', len(list(s.iter('testcase')))))
                           for s in root.findall('testsuite')) if root.tag == 'testsuites' else len(cases)
        if not cases or int(declared) != len(cases):
            raise ValueError('empty or incomplete report')
    except (OSError, ET.ParseError, ValueError) as error:
        return f'### {title}\n\nNot run/report missing or invalid: {escape(str(error))}\n', False

    counts = Counter()
    rosbags = Counter()
    reasons = Counter()
    skipped_names = set()
    problems = []
    nested_total = 0
    for case in cases:
        name = case.get('name', '(unnamed)')
        skipped = case.find('skipped')
        output = case.findtext('system-out', '')
        failed = case.find('failure') is not None or case.find('error') is not None
        # A module-level importorskip has name=test_rosbags and no classname;
        # executed cases carry the module in classname.
        module_parts = case.get('classname', '').split('.')
        if kind == 'pytest' and (name == 'test_rosbags' or 'test_rosbags' in module_parts):
            rosbags['failed' if failed else 'skipped' if skipped is not None else 'passed'] += 1
        if failed:
            counts['failed'] += 1
        elif skipped is not None:
            counts['skipped'] += 1
            skipped_names.add(name)
            reason = skipped.get('message') or skipped.text or 'no reason supplied'
            if kind == 'ctest':
                if name in DATASET_TESTS and 'static recording not found at ' in output:
                    reason = f'{name}: missing dataset (exit 77)'
                else:
                    reason = f'{name}: unexpected skip: {reason}; {output.strip()}'
                    problems.append(f'{name}: unexpected skip; inspect dependencies and fixtures')
            reasons[reason] += 1
        else:
            counts['passed'] += 1
        if kind == 'ctest':
            nested = sum(int(n) for n in re.findall(r'\b(\d+) skipped\b', output))
            nested_total += nested
            if nested and name not in DATASET_TESTS:
                problems.append(f'{name}: unexpected nested skipped cases: {nested}')
    if kind == 'ctest':
        missing = DATASET_TESTS - skipped_names
        if missing:
            problems.append(f'Expected absent-recording skips not reported: {sorted(missing)}')

    lines = [f'### {title}', '',
             f'{len(cases)} total: {counts["passed"]} passed, {counts["failed"]} failed, '
             f'{counts["skipped"]} skipped.', '']
    if kind == 'pytest':
        if rosbags['failed']:
            status = 'FAIL'
        elif rosbags['skipped'] or not rosbags['passed']:
            status = 'SKIPPED'
        else:
            status = 'PASS'
        detail = (f'{rosbags["passed"]} passed, {rosbags["failed"]} failed, '
                  f'{rosbags["skipped"]} skipped' if rosbags else 'not collected')
        lines += [f'rosbags independent CDR cross-check: **{status}** ({detail}).', '']
        if require_rosbags and status != 'PASS':
            problems.append('Required rosbags cross-check did not complete without failures or skips')
    if kind == 'ctest':
        lines += [f'Nested skipped cases: {nested_total} (separate from CTest entry totals).', '']
    lines += [f'- {count} × {escape(reason)}' for reason, count in sorted(reasons.items())]
    lines += ['', *[f'- ERROR: {escape(problem)}' for problem in problems], '']
    return '\n'.join(lines), not counts['failed'] and not problems


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    fixtures = sub.add_parser('fixtures')
    fixtures.add_argument('root', type=Path)
    summary = sub.add_parser('summary')
    summary.add_argument('reports', type=Path)
    summary.add_argument('--require-rosbags', action='store_true',
                         help='fail when the independent CDR cross-check is absent, skipped, or failed')
    args = parser.parse_args()
    if args.command == 'fixtures':
        check_fixtures(args.root)
        return 0
    ok = True
    for kind in ('pytest', 'ctest'):
        text, valid = summarize_report(args.reports / f'{kind}.xml', kind, args.require_rosbags)
        print(text)
        ok = ok and valid
    print('The initial spark/ pytest pass skips ROS and unset C++ executables. '
          'CTest subsequently runs the differential and recording-tool tests with built executables. '
          'Full static recordings are absent; their three CTest entries must skip explicitly. '
          'Missing dependencies or nested skips in other CTest entries fail this check. '
          'See verbose logs for individual case output; command outcomes remain authoritative.')
    return 0 if ok else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(str(error))
