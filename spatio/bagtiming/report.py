"""Plain Markdown rendering; no IO."""
from __future__ import annotations

DEFINITIONS = '''Receipt is MCAP log_time (integer ns), not publish_time. Header is sec*1e9+nanosec. No timestamp is shifted, repaired, or synthesized.

Gap: an inter-message interval greater than gap-factor × the median interval, in stored order. Median includes zero/negative intervals; a nonpositive median leaves the threshold undefined. Backward jump: any negative interval. Reset: a backward jump larger than 1 s, or a stamp dropping below 1.0e9 s after being above it. Large forward jumps use the gap rule. Gap examples are capped at 20; gap_count is complete. Rate is (count−1)/(last−first); null if fewer than two samples, no elapsed time, or any backward interval. Drift is least-squares slope of header-minus-receipt against receipt elapsed seconds, descriptive even across resets.

Scan start/end: min/max per-point absolute time. Without proven units AND origin, bounds and duration are null, never header stamps. Livox uses timebase + offset_time in ns. PointCloud2 field names, datatypes and magnitudes alone do not establish units/origin. An optional evidence contract selects one field and declares units and origin; floating point values retain their original quantization (reported ns are rounded to the nearest ns). Scan durations are max−min, not inter-scan periods. Point-order diagnostics describe storage order, which need not be firing order.

A scan is numerically covered only if an IMU header is ≤ start, one is ≥ end, and no IMU interval overlapping [start,end] exceeds gap-factor × median IMU period. This includes long intervals crossing either scan boundary. Missing/zero IMU headers, backward IMU clocks or a nonpositive median make coverage unknown. Unknown scan bounds and unsampled scans remain unknown, not covered. Percentages use evaluable scans as denominator, with total/unknown counts alongside. Numerical coverage does not establish physical synchronization. Clock domains are reported separately for receipt/header/points, with evidence; default unknown.

Lidar-versus-IMU header offsets use each lidar header minus the nearest IMU header, plus first-header offset. This cadence-dependent measurement is not a clock calibration. Odometry rates use each stream's own observed span and are reported for receipt and header separately; output/input count ratios do not prove backlog.

Records are processed in MCAP storage order (natural filename order across segments), never timestamp-sorted. A max-seconds slice stops at the first stored receipt beyond the limit from the bag summary start; it is not a complete discontinuity search. Full-bag metadata stays separate from analyzed counts. Per-point sampling leaves all selected message headers and receipts intact; point timelines then describe only the sampled sequence. TFMessage has no top-level header; nested transform clocks are not combined into a fabricated topic clock.
'''


def fmt(value):
    return 'unknown' if value is None else '{:.9g}'.format(value) if isinstance(value, float) else str(value)


def cell(value):
    return str(value).replace('|', '\\|').replace('\n', ' ')


def compatibility_rows(report):
    rows = []
    for topic in report['topics']:
        p = topic['points']
        cov = [x for x in report['imu_coverage'] if x['lidar_topic'] == topic['name']]
        coverage = '; '.join('{}: {}% ({}/{} evaluable, {} unknown)'.format(x['imu_topic'], fmt(x['coverage_percent_of_evaluable']), x['covered'], x['evaluable_scans'], x['unknown_scans']) for x in cov) or 'n/a'
        path = ('CustomMsg direct (lidar_type 1); copied FAST-LIO handler' if topic['type'].endswith('/CustomMsg')
                else 'adapter → PointCloud2 lidar_type 2; transformation and effective time-unit config unverified (x2)' if topic['name'] == '/slam/points'
                else 'IMU input: association/config must be checked' if topic['type'] == 'sensor_msgs/msg/Imu'
                else 'output/other; no input conversion established')
        clocks = '; '.join('{}={}: {}'.format(k, v['domain'], v['evidence']) for k, v in topic['clocks'].items() if k != 'points' or p)
        unknown = []
        if topic['clock_domain'] == 'unknown':
            unknown.append('header clock provenance')
        if p and p['scans_with_bounds'] != p['scans_total']:
            unknown.append('some/all scan bounds and coverage')
        if p and p['unit'] == 'unknown':
            unknown.append('point units/origin')
        unknown.append('physical synchronization and FAST-LIO runtime compatibility not proven')
        rows.append([topic['name'], topic['type'], 'receipt ns; header sec+ns' + ('; points ' + p['unit'] if p else ''), clocks,
                     p['timing_source'] if p else 'n/a', fmt(p['scan_duration_s']['median']) + ' s median' if p else 'n/a', coverage, path, '; '.join(unknown)])
    return rows


def markdown(report):
    lines = ['# Bag timing inspection', '', 'Bag: `{}`'.format(report['bag']), '',
             'Reader: {}. Analyzed {} messages. Point sampling every {} scan(s).'.format(report['reader'], report['analysis']['message_count'], report['point_timing_sampled_every']), '',
             'These are timestamp diagnostics, not a physical-accuracy or FAST-LIO acceptance result.', '',
             '## Compatibility table', '',
             '| Topic | Type | Time unit | Clock domain + evidence | Point timing | Scan duration | IMU coverage | FAST-LIO input path | Unknowns |',
             '|---|---|---|---|---|---|---|---|---|']
    lines += ['| ' + ' | '.join(cell(v) for v in row) + ' |' for row in compatibility_rows(report)]
    lines += ['', '## Definitions and limits', '', DEFINITIONS, '## Measured topic diagnostics', '']
    for t in report['topics']:
        lines += ['### ' + t['name'], '', 'Type: `{}`; count: {}; frames: `{}`.'.format(t['type'], t['count'], ', '.join(t['frame_ids'])), '',
                  'Message fields: ' + '; '.join('`' + f + '`' for f in t['fields']) + '.', '']
        for basis in ('receipt', 'header'):
            tm = t[basis]
            lines.append('{}: first {} ns; last {} ns; rate {} Hz; max gap {} s; {} gaps; {} backward jumps; {} resets; {} zeros.'.format(basis, fmt(tm['first_ns']), fmt(tm['last_ns']), fmt(tm['rate_hz']), fmt(tm['max_gap_s']), tm['gap_count'], tm['backward_jumps'], len(tm['resets']), tm['zero_stamps']))
        lines += ['', 'Header minus receipt (s): ' + '; '.join(k + '=' + fmt(v) for k, v in t['header_minus_receipt_s'].items()) + '.', '']
        p = t['points']
        if p:
            lines += ['Point units evidence: ' + p['unit_evidence'], '',
                      'Scan duration (s): ' + '; '.join(k + '=' + fmt(v) for k, v in p['scan_duration_s'].items()), '',
                      'Monotonic storage fraction: {}. Sampled: {}/{}; absolute bounds: {}.'.format(fmt(p['monotonic_within_scan_fraction']), p['scans_sampled'], p['scans_total'], p['scans_with_bounds']), '']
            for f in p['timing_fields']:
                lines.append('Point field {} {} count={}: raw min {}, max {}; nonfinite {}.'.format(f['name'], f['datatype'], f['count'], fmt(f['min']), fmt(f['max']), f['nonfinite_count']))
            lines.append('')
    lines += ['## Lidar versus IMU header offsets', '']
    for x in report['lidar_imu_header_offsets']:
        lines.append('{} minus {}: first {} s; nearest-header distribution {}. {}'.format(x['lidar_topic'], x['imu_topic'], fmt(x['first_header_offset_s']), x['nearest_header_offset_s'], x['method']))
    lines += ['', '## FAST-LIO output versus input', '']
    for x in report['odometry_vs_scan']:
        lines.append('{} vs {} ({}): input {} Hz ({}), output {} Hz ({}); rate ratio {}; count ratio {}.'.format(x['lidar_topic'], x['odometry_topic'], x['basis'], fmt(x['input_rate_hz']), x['input_count'], fmt(x['output_rate_hz']), x['output_count'], fmt(x['rate_ratio']), fmt(x['count_ratio'])))
    lines += ['', '## Findings', '']
    lines += ['- **{} {}**: {}'.format(x['severity'], x['code'], x['text']) for x in report['findings']]
    return '\n'.join(lines) + '\n'
