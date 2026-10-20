"""Pure timing analysis of (topic, receipt_ns, decoded_msg_like) records.

Messages may be mappings or mcap_ros2 dynamic objects. No file IO here.
PointCloud2 units AND origin require an explicit, evidence-bearing contract.
"""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from fractions import Fraction
import math
import statistics

import numpy as np

NS = 1_000_000_000
LIVOX_EVIDENCE = (
    'Copied livox_ros_driver2 @4a1def9 msg/CustomMsg.msg: uint64 timebase '
    '(first point); msg/CustomPoint.msg: uint32 offset_time (relative to base). '
    'Units are not specified in the .msg comments. src/lddc.cpp:367-373 sets '
    'header from timebase using ROS2 rclcpp::Time and ROS1 timebase/1e9; '
    '395 subtracts pkg.base_time for offset_time. FAST_LIO_ROS2 @2fffc57 '
    'src/preprocess.cpp:180-181 converts offset_time/1e6 to milliseconds. '
    'These copied sources establish ns; correspondence to the running publisher '
    'cannot be established from the schema alone.'
)
DATATYPES = {1: ('INT8', 'i1'), 2: ('UINT8', 'u1'), 3: ('INT16', 'i2'),
             4: ('UINT16', 'u2'), 5: ('INT32', 'i4'), 6: ('UINT32', 'u4'),
             7: ('FLOAT32', 'f4'), 8: ('FLOAT64', 'f8')}
TIMING_NAMES = {'time', 't', 'timestamp', 'offset_time'}


def get(obj, key, default=None):
    return obj.get(key, default) if isinstance(obj, dict) else getattr(obj, key, default)


def header_ns(msg):
    h = get(msg, 'header')
    stamp = get(h, 'stamp')
    if stamp is None:
        return None
    sec, nano = int(get(stamp, 'sec')), int(get(stamp, 'nanosec'))
    if not 0 <= nano < NS:
        raise ValueError('header.stamp.nanosec outside [0, 1e9)')
    return sec * NS + nano


def distribution(values):
    a = np.asarray(values, dtype=np.float64)
    if not len(a):
        return dict.fromkeys(('median', 'p5', 'p95', 'min', 'max'))
    return {'median': float(np.median(a)), 'p5': float(np.percentile(a, 5)),
            'p95': float(np.percentile(a, 95)), 'min': float(a.min()), 'max': float(a.max())}


def timeline(values, gap_factor=3.0):
    """Intervals retain record order; median includes zero/negative intervals.

    A nonpositive median makes the gap threshold undefined. Resets do not
    disappear by sorting. Rate is observed span rate, null across regressions.
    """
    deltas = [b - a for a, b in zip(values, values[1:])]
    median = statistics.median(deltas) if deltas else None
    threshold = median * gap_factor if median is not None and median > 0 else None
    gap_indices = [i for i, d in enumerate(deltas) if threshold is not None and d > threshold]
    resets = [{'index': i + 1, 'from_ns': a, 'to_ns': b}
              for i, (a, b) in enumerate(zip(values, values[1:]))
              if a - b > NS or (a > 10**18 and b < 10**18)]
    backwards = sum(d < 0 for d in deltas)
    span = values[-1] - values[0] if len(values) >= 2 else 0
    return {'first_ns': values[0] if values else None, 'last_ns': values[-1] if values else None,
            'rate_hz': (len(values) - 1) * NS / span if span > 0 and not backwards else None,
            'median_period_s': median / NS if median is not None and median > 0 else None,
            'max_gap_s': max([0] + deltas) / NS if deltas else None,
            'gap_count': len(gap_indices),
            'gaps': [[(values[i + 1] - values[0]) / NS, deltas[i] / NS] for i in gap_indices[:20]],
            'backward_jumps': backwards, 'resets': resets, 'zero_stamps': values.count(0),
            'duplicate_intervals': deltas.count(0)}


def offset_stats(receipts, headers):
    offsets = [(h - r) / NS for r, h in zip(receipts, headers)]
    out = distribution(offsets)
    slope = None
    if len(receipts) >= 2:
        x = np.array([(r - receipts[0]) / NS for r in receipts])
        y = np.array(offsets)
        x = x - x.mean()
        denom = float(np.dot(x, x))
        if denom:
            slope = float(np.dot(x, y - y.mean()) / denom)
    out['drift_slope_s_per_s'] = slope
    return out


def pointcloud_fields(msg):
    """Zero-copy strided field views respect endian, row padding and field count."""
    width, height = int(get(msg, 'width')), int(get(msg, 'height'))
    step, row = int(get(msg, 'point_step')), int(get(msg, 'row_step'))
    data = get(msg, 'data')
    if not isinstance(data, (bytes, bytearray, memoryview)):
        data = bytes(data)
    if min(width, height, step, row) < 0 or row < width * step or len(data) != row * height:
        raise ValueError('PointCloud2 dimensions/row_step/data length inconsistent')
    result = []
    for f in get(msg, 'fields', []):
        name = get(f, 'name')
        if name not in TIMING_NAMES:
            continue
        datatype, count, offset = int(get(f, 'datatype')), int(get(f, 'count')), int(get(f, 'offset'))
        if datatype not in DATATYPES or count < 1:
            raise ValueError('Invalid timing PointField datatype/count')
        label, code = DATATYPES[datatype]
        dtype = np.dtype(('>' if get(msg, 'is_bigendian', False) else '<') + code)
        if offset < 0 or offset + count * dtype.itemsize > step:
            raise ValueError('Timing PointField extends outside point_step')
        a = (np.ndarray((height, width, count), dtype=dtype, buffer=data, offset=offset,
                        strides=(row, step, dtype.itemsize)).reshape(-1)
             if width * height else np.array([], dtype=dtype))
        finite = a[np.isfinite(a)]
        result.append({'name': name, 'datatype': label, 'datatype_id': datatype, 'count': count,
                       'offset': offset, 'values': a,
                       'min': finite.min().item() if len(finite) else None,
                       'max': finite.max().item() if len(finite) else None,
                       'nonfinite_count': int(len(a) - len(finite))})
    return result


def to_ns(value, unit):
    # Preserve integer ns and subtract epoch values before float conversion.
    return round(Fraction(value.item() if hasattr(value, 'item') else value) * (NS if unit == 's' else 1))


def scan_summary(msg, msg_type, contract=None, gap_factor=3.0):
    h = header_ns(msg)
    out = {'start_ns': None, 'end_ns': None, 'duration_s': None, 'monotonic': None,
           'backward_jumps_within_scan': 0, 'resets_within_scan': 0,
           'forward_gaps_within_scan': 0, 'max_forward_gap_ns': None,
           'header_minus_first_point_s': None, 'fields': [], 'timebase_ns': None,
           'timing_source': 'none', 'unit': 'unknown', 'unit_evidence': 'No per-point timing.'}
    if msg_type == 'livox_ros_driver2/msg/CustomMsg':
        base = int(get(msg, 'timebase'))
        if not 0 <= base < 2**64:
            raise ValueError('CustomMsg.timebase outside uint64')
        points = get(msg, 'points', [])
        if get(msg, 'point_num', len(points)) != len(points):
            raise ValueError('CustomMsg.point_num differs from points length')
        offsets = [int(get(p, 'offset_time')) for p in points]
        if any(x < 0 or x >= 2**32 for x in offsets):
            raise ValueError('CustomPoint.offset_time outside uint32')
        raw = np.asarray(offsets, dtype=np.int64)
        absolute_origin, unit = base, 'ns'
        out.update(timing_source='CustomMsg.timebase+offset_time', unit='ns',
                   unit_evidence=LIVOX_EVIDENCE, timebase_ns=base)
        out['fields'] = [{'name': 'offset_time', 'datatype': 'UINT32', 'count': 1,
                          'min': min(offsets) if offsets else None, 'max': max(offsets) if offsets else None,
                          'nonfinite_count': 0}]
    else:
        fields = pointcloud_fields(msg)
        out['fields'] = [{k: v for k, v in f.items() if k != 'values'} for f in fields]
        out['timing_source'] = '; '.join('PointCloud2 field {} {}'.format(f['name'], f['datatype']) for f in fields) or 'none'
        out['unit_evidence'] = ('Field names/datatypes/ranges do not establish units or absolute/relative origin.'
                                if fields else 'No per-point timing field.')
        if not contract:
            return out
        if (contract.get('unit') not in ('s', 'ns') or contract.get('origin') not in ('header', 'absolute')
                or not contract.get('evidence', '').strip()):
            raise ValueError('Timing contract requires unit s/ns, origin header/absolute, and evidence')
        matches = [f for f in fields if f['name'] == contract.get('field')]
        if len(matches) != 1 or matches[0]['count'] != 1:
            raise ValueError('Timing contract field missing, duplicated, or not scalar')
        field = matches[0]
        if field['nonfinite_count']:
            out['unit_evidence'] += ' Nonfinite values prevent scan bounds.'
            return out
        raw = field['values']
        unit = contract['unit']
        absolute_origin = h if contract['origin'] == 'header' else 0
        out.update(unit=unit, unit_evidence=contract['evidence'], timing_source='PointCloud2 field {} {}'.format(field['name'], field['datatype']))
        if absolute_origin is None:
            return out
    if not len(raw):
        return out
    start, end = absolute_origin + to_ns(raw.min(), unit), absolute_origin + to_ns(raw.max(), unit)
    # NumPy summaries avoid constructing Python objects for every point interval.
    # Integer offsets stay exact. Floating differences retain wire quantization.
    deltas = np.diff(raw.astype(np.int64, copy=False) if raw.dtype.kind in 'iu' else raw.astype(np.float64, copy=False))
    scale = NS if unit == 's' else 1
    med = float(np.median(deltas)) if len(deltas) else None
    out.update(start_ns=start, end_ns=end, duration_s=(end - start) / NS,
               monotonic=bool(np.all(raw[1:] >= raw[:-1])),
               backward_jumps_within_scan=int(np.count_nonzero(deltas < 0)),
               resets_within_scan=int(np.count_nonzero((deltas < -NS / scale) |
                   ((raw[:-1] > (10**18 - absolute_origin) / scale) &
                    (raw[1:] < (10**18 - absolute_origin) / scale)))),
               forward_gaps_within_scan=int(np.count_nonzero(deltas > med * gap_factor)) if med is not None and med > 0 else 0,
               max_forward_gap_ns=to_ns(max(0, deltas.max().item()), unit) if len(deltas) else None,
               header_minus_first_point_s=(h - start) / NS if h is not None else None)
    return out


def clock_spec(info, basis):
    supplied = info.get('clocks', {}).get(basis)
    if supplied:
        if supplied.get('domain') not in ('sensor', 'host', 'postprocess-receipt', 'unknown') or not supplied.get('evidence'):
            raise ValueError('Clock domain requires allowed domain and evidence')
        return dict(supplied)
    return {'domain': 'unknown', 'evidence': 'No acquisition clock provenance established for this time base; numerical overlap is insufficient.'}


def analyze(records, topic_info, gap_factor=3.0, timing_contracts=None, point_timing_sampled_every=1):
    """Streaming input; retain timestamps and per-scan summaries only, not messages.

    topic_info maps topics to type/fields/schema/clocks evidence. Optional
    timing_contracts map PointCloud2 topics to field/unit/origin/evidence.
    Reader may replace unsampled scans with header-only messages carrying
    _point_timing_skipped=True; they still contribute all header/receipt stats.
    """
    if not math.isfinite(gap_factor) or gap_factor <= 0 or point_timing_sampled_every < 1:
        raise ValueError('gap factor and sampling interval must be positive')
    states = {}
    findings = []

    def finding(severity, code, text):
        findings.append({'severity': severity, 'code': code, 'text': text})

    for topic, receipt, msg in records:
        info = topic_info[topic]
        kind = info['type']
        lidar = kind in ('livox_ros_driver2/msg/CustomMsg', 'sensor_msgs/msg/PointCloud2')
        if topic not in states:
            states[topic] = {'receipts': [], 'headers': [], 'paired_receipts': [], 'frames': set(),
                             'scans': [], 'missing_header': 0, 'sampled': 0, 'pointfields': {}}
        s = states[topic]
        index = len(s['receipts'])
        s['receipts'].append(int(receipt))
        h = header_ns(msg)
        for transform in get(msg, 'transforms', []):
            frame = get(get(transform, 'header'), 'frame_id')
            if frame is not None:
                s['frames'].add(str(frame))
        if h is None:
            s['missing_header'] += 1
        else:
            s['headers'].append(h)
            s['paired_receipts'].append(int(receipt))
            s['frames'].add(str(get(get(msg, 'header'), 'frame_id', '')))
        if lidar:
            if index % point_timing_sampled_every or get(msg, '_point_timing_skipped', False):
                s['scans'].append({'index': index, 'start_ns': None, 'end_ns': None, 'sampled': False})
                continue
            scan = scan_summary(msg, kind, (timing_contracts or {}).get(topic), gap_factor)
            scan.update(index=index, sampled=True)
            s['scans'].append(scan)
            s['sampled'] += 1
            for f in scan['fields']:
                key = (f['name'], f['datatype'], f['count'])
                if key not in s['pointfields']:
                    s['pointfields'][key] = dict(f)
                else:
                    prev = s['pointfields'][key]
                    for key2, fn in [('min', min), ('max', max)]:
                        if f[key2] is not None:
                            prev[key2] = fn(prev[key2], f[key2]) if prev[key2] is not None else f[key2]
                    prev['nonfinite_count'] += f['nonfinite_count']

    topics = []
    for name, s in states.items():
        info = topic_info[name]
        clocks = {basis: clock_spec(info, basis) for basis in ('receipt', 'header', 'points')}
        t = {'name': name, 'type': info['type'], 'count': len(s['receipts']),
             'frame_ids': sorted(s['frames']), 'fields': info.get('fields', []),
             'schema_definitions': info.get('schema_definitions', []),
             'receipt': timeline(s['receipts'], gap_factor), 'header': timeline(s['headers'], gap_factor),
             'missing_header_count': s['missing_header'], 'clocks': clocks,
             'clock_domain': clocks['header']['domain'], 'clock_domain_evidence': clocks['header']['evidence'],
             'header_minus_receipt_s': offset_stats(s['paired_receipts'], s['headers']), 'points': None}
        for basis in ('receipt', 'header'):
            time = t[basis]
            for key, code in [('backward_jumps', 'BACKWARD_JUMP'), ('gap_count', 'TIME_GAP'), ('zero_stamps', 'ZERO_STAMP')]:
                if time[key]:
                    finding('warn', code, '{} {}: {} {}; max forward interval {} s.'.format(name, basis, time[key], key, time['max_gap_s']))
            if time['resets']:
                finding('error', 'CLOCK_RESET', '{} {}: {} resets.'.format(name, basis, len(time['resets'])))
        if s['missing_header']:
            finding('info', 'NO_HEADER', '{}: {} messages have no top-level header.'.format(name, s['missing_header']))
        if s['scans']:
            scanned = [x for x in s['scans'] if x['sampled']]
            first = scanned[0] if scanned else {}
            bounds = [x for x in scanned if x['start_ns'] is not None]
            p = {'timing_source': '; '.join(sorted(set(x['timing_source'] for x in scanned))) or 'none', 'unit': first.get('unit', 'unknown'),
                 'unit_evidence': first.get('unit_evidence', 'No sampled scans'),
                 'timing_sources': sorted(set(x['timing_source'] for x in scanned)),
                 'timing_fields': list(s['pointfields'].values()),
                 'scan_duration_s': distribution([x['duration_s'] for x in bounds]),
                 'monotonic_within_scan_fraction': sum(x['monotonic'] for x in bounds) / len(bounds) if bounds else None,
                 'header_minus_first_point_s': distribution([x['header_minus_first_point_s'] for x in bounds if x['header_minus_first_point_s'] is not None]) if bounds else None,
                 'scans_total': len(s['scans']), 'scans_sampled': len(scanned), 'scans_with_bounds': len(bounds),
                 'point_timing_sampled_every': point_timing_sampled_every,
                 'scan_bounds': [{'index': x['index'], 'start_ns': x['start_ns'], 'end_ns': x['end_ns'], 'sampled': x['sampled']} for x in s['scans']],
                 'scan_start': timeline([x['start_ns'] for x in bounds], gap_factor),
                 'scan_end': timeline([x['end_ns'] for x in bounds], gap_factor),
                 'timebase': timeline([x['timebase_ns'] for x in scanned if x['timebase_ns'] is not None], gap_factor),
                 'backward_jumps_within_scan': sum(x['backward_jumps_within_scan'] for x in scanned),
                 'resets_within_scan': sum(x['resets_within_scan'] for x in scanned),
                 'forward_gaps_within_scan': sum(x['forward_gaps_within_scan'] for x in scanned),
                 'max_forward_gap_within_scan_s': max([x['max_forward_gap_ns'] or 0 for x in scanned], default=0) / NS}
            t['points'] = p
            unknown = len(scanned) - len(bounds)
            if unknown:
                finding('warn', 'UNKNOWN_SCAN_BOUNDS', '{}: {}/{} sampled scans lack defensible absolute point times; scan bounds/coverage unknown, never replaced with header.'.format(name, unknown, len(scanned)))
            if not s['pointfields']:
                finding('warn', 'NO_POINT_TIMING', '{}: no per-point timing in {} sampled scans.'.format(name, len(scanned)))
            elif p['unit'] == 'unknown':
                finding('warn', 'UNKNOWN_TIME_UNIT', '{}: {} timing field variants; units/origin unproved.'.format(name, len(s['pointfields'])))
            for basis in ('scan_start', 'scan_end', 'timebase'):
                tm = p[basis]
                if tm['backward_jumps'] or tm['gap_count']:
                    finding('warn', 'POINT_TIME_DISCONTINUITY', '{} {}: {} backward jumps, {} resets, {} large forward intervals; max {} s.'.format(name, basis, tm['backward_jumps'], len(tm['resets']), tm['gap_count'], tm['max_gap_s']))
            if p['backward_jumps_within_scan'] or p['forward_gaps_within_scan']:
                finding('warn', 'POINT_STORAGE_ORDER', '{}: {} negative point intervals, {} resets >1 s, {} large forward intervals in storage order; nonmonotonic storage is not by itself a sensor clock fault.'.format(name, p['backward_jumps_within_scan'], p['resets_within_scan'], p['forward_gaps_within_scan']))
        if t['header_minus_receipt_s']['median'] is not None:
            finding('info', 'HEADER_RECEIPT_OFFSET', '{}: header minus receipt median {:.9g} s; p5 {:.9g}, p95 {:.9g}, min {:.9g}, max {:.9g}; slope {} s/s. No correction applied.'.format(name, *[t['header_minus_receipt_s'][k] for k in ('median', 'p5', 'p95', 'min', 'max', 'drift_slope_s_per_s')]))
        if clocks['header']['domain'] == 'unknown' and s['headers']:
            finding('warn', 'UNKNOWN_CLOCK_DOMAIN', '{}: header clock domain unknown for {} stamps. Timestamp overlap does not prove physical synchronization.'.format(name, len(s['headers'])))
        topics.append(t)

    coverage, offsets, rates = [], [], []
    imus = [t for t in topics if t['type'] == 'sensor_msgs/msg/Imu']
    lidars = [t for t in topics if t['points'] is not None]
    for lidar in lidars:
        if not imus:
            finding('warn', 'NO_IMU_TOPIC', '{}: {} scans; no IMU topic inspected, coverage unavailable.'.format(lidar['name'], lidar['count']))
        for imu in imus:
            stamps = states[imu['name']]['headers']
            period = imu['header']['median_period_s']
            valid = (period is not None and not imu['header']['backward_jumps']
                     and not imu['missing_header_count'] and not imu['header']['zero_stamps'])
            threshold = period * NS * gap_factor if valid else None
            bad = [i for i, (a, b) in enumerate(zip(stamps, stamps[1:])) if valid and b - a > threshold]
            bad_starts = [stamps[i] for i in bad]
            bad_ends = [stamps[i + 1] for i in bad]
            covered = evaluable = 0
            examples = []
            for scan in states[lidar['name']]['scans']:
                start, end = scan['start_ns'], scan['end_ns']
                if start is None or not valid:
                    continue
                evaluable += 1
                ok = bool(stamps and stamps[0] <= start and stamps[-1] >= end)
                # Any long interval overlapping the scan, including crossing its boundaries, fails.
                j = bisect_right(bad_ends, start)
                if j < len(bad_starts) and bad_starts[j] < end:
                    ok = False
                if ok:
                    covered += 1
                elif len(examples) < 20:
                    examples.append(scan['index'])
            count = len(states[lidar['name']]['scans'])
            item = {'lidar_topic': lidar['name'], 'imu_topic': imu['name'], 'scans': count,
                    'evaluable_scans': evaluable, 'covered': covered, 'unknown_scans': count - evaluable,
                    'coverage_percent_of_evaluable': covered * 100 / evaluable if evaluable else None,
                    'uncovered_examples': examples, 'imu_median_period_s': period,
                    'meaning': 'Numerical header coverage only; physical clock synchronization unverified.'}
            coverage.append(item)
            if count != covered:
                finding('warn', 'IMU_COVERAGE', '{} vs {}: {}/{} evaluable scans covered, {} unknown of {} total.'.format(lidar['name'], imu['name'], covered, evaluable, count - evaluable, count))
            # Nearest header difference is bounded by cadence and is NOT a clock calibration.
            sorted_imu = sorted(stamps)
            delta = []
            if sorted_imu:
                for h in states[lidar['name']]['headers']:
                    i = bisect_left(sorted_imu, h)
                    candidates = sorted_imu[max(0, i - 1):i + 1]
                    nearest = min(candidates, key=lambda v: abs(h - v))
                    delta.append((h - nearest) / NS)
            offsets.append({'lidar_topic': lidar['name'], 'imu_topic': imu['name'],
                            'nearest_header_offset_s': distribution(delta),
                            'first_header_offset_s': (states[lidar['name']]['headers'][0] - stamps[0]) / NS if stamps and states[lidar['name']]['headers'] else None,
                            'method': 'lidar header minus nearest IMU header; not a sensor synchronization estimate; sorted only for nearest lookup'})
        for odom in [t for t in topics if t['name'] == '/Odometry']:
            for basis in ('receipt', 'header'):
                a, b = odom[basis]['rate_hz'], lidar[basis]['rate_hz']
                rates.append({'lidar_topic': lidar['name'], 'odometry_topic': odom['name'], 'basis': basis,
                              'input_count': lidar['count'], 'output_count': odom['count'],
                              'input_rate_hz': b, 'output_rate_hz': a,
                              'rate_ratio': a / b if a is not None and b else None,
                              'count_ratio': odom['count'] / lidar['count']})
            finding('info', 'ODOMETRY_VS_SCAN', '{}: {} input scans, {} /Odometry outputs; count ratio {:.6g}. Rate comparison uses each stream span; counts alone do not establish backlog.'.format(lidar['name'], lidar['count'], odom['count'], odom['count'] / lidar['count']))
    if point_timing_sampled_every > 1:
        finding('warn', 'POINT_TIMING_SAMPLED', 'Per-point timing decoded every {}th scan; every selected message header/receipt retained. Unsampled bounds/coverage unknown; point discontinuities apply only to sampled sequence.'.format(point_timing_sampled_every))
    return {'schema': 'spatio.bagtiming/1', 'topics': topics, 'imu_coverage': coverage,
            'lidar_imu_header_offsets': offsets, 'odometry_vs_scan': rates,
            'gap_factor': gap_factor, 'point_timing_sampled_every': point_timing_sampled_every, 'findings': findings}
