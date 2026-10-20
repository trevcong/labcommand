from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from mcap_ros2.writer import Writer
from mcap_ros2.decoder import DecoderFactory
from mcap.reader import make_reader

from spatio.bagtiming.core import NS, analyze, scan_summary, timeline
from spatio.bagtiming.reader import CDR, BagReader, manual_custommsg

HEADER_DEPS = '''
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
IMU_SCHEMA = '''std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
''' + HEADER_DEPS + '''
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
'''
PC_SCHEMA = '''std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
''' + HEADER_DEPS + '''
================================================================================
MSG: sensor_msgs/PointField
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count
'''
CUSTOM_SCHEMA = '''std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/CustomPoint[] points
''' + HEADER_DEPS + '''
================================================================================
MSG: livox_ros_driver2/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
'''
RECORDED_CUSTOM_SCHEMA = (Path(__file__).parent / 'fixtures' /
                          'mid360_recorded_custommsg.ros2msg').read_text()
IMU, PC, CUSTOM = 'sensor_msgs/msg/Imu', 'sensor_msgs/msg/PointCloud2', 'livox_ros_driver2/msg/CustomMsg'


def header(t, frame='sensor'):
    return {'stamp': {'sec': t // NS, 'nanosec': t % NS}, 'frame_id': frame}


def imu(t):
    return {'header': header(t), 'orientation': dict(x=0., y=0., z=0., w=1.),
            'orientation_covariance': [0.] * 9, 'angular_velocity': dict(x=0., y=0., z=0.),
            'angular_velocity_covariance': [0.] * 9, 'linear_acceleration': dict(x=0., y=0., z=0.),
            'linear_acceleration_covariance': [0.] * 9}


def custom(t, offsets=(0, 100_000_000)):
    return {'header': header(t), 'timebase': t, 'point_num': len(offsets), 'lidar_id': 1, 'rsvd': [0, 0, 0],
            'points': [dict(offset_time=x, x=1., y=2., z=3., reflectivity=4, tag=0, line=1) for x in offsets]}


def cloud(t, values=(0., .1), name='time', datatype=7, endian=False):
    fmts = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
    size = struct.calcsize(fmts[datatype])
    data = b''.join(struct.pack(('>' if endian else '<') + fmts[datatype], v) for v in values)
    return {'header': header(t), 'height': 1, 'width': len(values),
            'fields': [{'name': name, 'offset': 0, 'datatype': datatype, 'count': 1}],
            'is_bigendian': endian, 'point_step': size, 'row_step': len(data), 'data': data, 'is_dense': True}


def info(**names):
    return {topic: {'type': kind, 'fields': ['std_msgs/Header header']} for topic, kind in names.items()}


def contract(name='time', unit='s', origin='header'):
    return dict(field=name, unit=unit, origin=origin, evidence='Synthetic fixture producer explicitly encodes ' + unit + ' with origin ' + origin)


def write_bag(path, custom_points=False, backwards=False):
    with path.open('wb') as f:
        w = Writer(f)
        i = w.register_msgdef(IMU, IMU_SCHEMA)
        p = w.register_msgdef(PC, PC_SCHEMA)
        for n in range(5):
            t = 10 * NS + n * 50_000_000
            w.write_message('/imu', i, imu(t), log_time=t if not backwards or n != 4 else 9 * NS)
            if n in (0, 2):
                w.write_message('/points', p, cloud(t), log_time=t + 1)
        if custom_points:
            c = w.register_msgdef(CUSTOM, CUSTOM_SCHEMA)
            for n in range(3):
                w.write_message('/livox/lidar', c, custom(10 * NS + n * 100_000_000), log_time=10 * NS + n * 100_000_000)
        w.finish()


class TimingTests(unittest.TestCase):
    def test_receipt_and_header_reported_separately(self):
        r = analyze([('/imu', 10 * NS, imu(NS)), ('/imu', 12 * NS, imu(2 * NS))], info(**{'/imu': IMU}))
        t = r['topics'][0]
        self.assertEqual(t['receipt']['first_ns'], 10 * NS)
        self.assertEqual(t['header']['first_ns'], NS)
        self.assertEqual(t['receipt']['rate_hz'], .5)
        self.assertEqual(t['header']['rate_hz'], 1.)

    def test_backward_jump_detected(self):
        x = timeline([10 * NS, 11 * NS, 10 * NS + 5])
        self.assertEqual(x['backward_jumps'], 1)
        self.assertIsNone(x['rate_hz'])
        self.assertEqual(x['resets'], [])

    def test_clock_reset_to_small_values_detected(self):
        x = timeline([1_700_000_000 * NS, 4 * NS])
        self.assertEqual(x['resets'], [{'index': 1, 'from_ns': 1_700_000_000 * NS, 'to_ns': 4 * NS}])
        # Epoch threshold crossing is a reset even if the drop is <1 second.
        self.assertEqual(len(timeline([10**18 + 1, 10**18 - 1])['resets']), 1)

    def test_gap_detected_with_factor(self):
        x = timeline([0, NS, 2 * NS, 6 * NS], 3)
        self.assertEqual(x['gaps'], [[6., 4.]])
        self.assertEqual(x['gap_count'], 1)
        self.assertEqual(timeline([0, NS, 2 * NS, 6 * NS], 4)['gap_count'], 0)

    def test_constant_header_receipt_offset_reported_not_corrected(self):
        records = [('/imu', (10 + i) * NS + 23, imu((8 + i) * NS + 23)) for i in range(5)]
        frozen = copy.deepcopy(records)
        out = analyze(records, info(**{'/imu': IMU}))['topics'][0]
        self.assertEqual(records, frozen)
        self.assertEqual(out['header_minus_receipt_s']['median'], -2)
        self.assertEqual(out['header_minus_receipt_s']['min'], -2)
        self.assertEqual(out['header_minus_receipt_s']['drift_slope_s_per_s'], 0)
        self.assertEqual(out['header']['first_ns'], 8 * NS + 23)

    def test_custommsg_point_time_is_timebase_plus_offset_ns(self):
        out = scan_summary(custom(1_700_000_000 * NS + 17, [5, 19]), CUSTOM)
        self.assertEqual(out['start_ns'], 1_700_000_000 * NS + 22)
        self.assertEqual(out['end_ns'], 1_700_000_000 * NS + 36)
        self.assertEqual(out['unit'], 'ns')
        self.assertEqual(out['header_minus_first_point_s'], -5e-9)

    def test_pointcloud2_relative_float32_time_field(self):
        out = scan_summary(cloud(100 * NS, [0., .125]), PC, contract())
        self.assertEqual(out['start_ns'], 100 * NS)
        self.assertEqual(out['end_ns'], 100 * NS + 125_000_000)
        self.assertEqual(out['fields'][0]['datatype'], 'FLOAT32')
        self.assertEqual(out['duration_s'], .125)

    def test_pointcloud2_absolute_float64_timestamp_field(self):
        out = scan_summary(cloud(1_700_000_000 * NS, [1700000000., 1700000000.125], 'timestamp', 8), PC,
                           contract('timestamp', 's', 'absolute'))
        self.assertEqual(out['start_ns'], 1_700_000_000 * NS)
        self.assertEqual(out['end_ns'], 1_700_000_000 * NS + 125_000_000)
        self.assertEqual(out['fields'][0]['datatype'], 'FLOAT64')
        self.assertEqual(out['duration_s'], .125)

    def test_unknown_time_unit_reported_unknown_not_guessed(self):
        for name, datatype, values in [('time', 7, [0., .1]), ('timestamp', 8, [1700000000., 1700000000.1]), ('t', 6, [0, 100000])]:
            r = analyze([('/points', NS, cloud(NS, values, name, datatype))], info(**{'/points': PC}))
            p = r['topics'][0]['points']
            self.assertEqual(p['unit'], 'unknown')
            self.assertIsNone(p['scan_bounds'][0]['start_ns'])
            self.assertIsNotNone(p['timing_fields'][0]['max'])
            self.assertIn('UNKNOWN_TIME_UNIT', [f['code'] for f in r['findings']])

    def test_no_point_timing_gives_null_scan_bounds_and_finding(self):
        msg = cloud(NS, name='x')
        r = analyze([('/points', NS, msg)], info(**{'/points': PC}))
        p = r['topics'][0]['points']
        self.assertEqual(p['timing_source'], 'none')
        self.assertIsNone(p['scan_bounds'][0]['start_ns'])
        self.assertIsNone(p['scan_bounds'][0]['end_ns'])
        self.assertIsNone(p['header_minus_first_point_s'])
        self.assertIn('NO_POINT_TIMING', [f['code'] for f in r['findings']])

    def test_scan_duration_from_point_times(self):
        msg = custom(100 * NS, [30_000_000, 0, 90_000_000])
        out = scan_summary(msg, CUSTOM)
        self.assertEqual(out['duration_s'], .09)
        self.assertFalse(out['monotonic'])
        self.assertEqual(out['backward_jumps_within_scan'], 1)

    def coverage(self, stamps, base=NS, offsets=(0, 100_000_000)):
        records = [('/lidar', base, custom(base, offsets))] + [('/imu', x, imu(x)) for x in stamps]
        return analyze(records, info(**{'/imu': IMU, '/lidar': CUSTOM}))['imu_coverage'][0]

    def test_imu_coverage_full(self):
        c = self.coverage([NS + i * 10_000_000 for i in range(11)])
        self.assertEqual(c['covered'], 1)
        self.assertEqual(c['coverage_percent_of_evaluable'], 100.)

    def test_imu_coverage_internal_gap_marks_uncovered(self):
        c = self.coverage([NS + i * 10_000_000 for i in [0, 1, 2, 3, 8, 9, 10]])
        self.assertEqual(c['covered'], 0)
        self.assertEqual(c['uncovered_examples'], [0])

    def test_imu_starting_after_first_scan_marks_uncovered(self):
        c = self.coverage([NS + i * 10_000_000 for i in range(1, 12)])
        self.assertEqual(c['covered'], 0)
        self.assertEqual(c['evaluable_scans'], 1)

    def test_odometry_rate_vs_scan_rate_reported(self):
        records = [('/lidar', NS + i * NS // 10, custom(NS + i * NS // 10)) for i in range(11)]
        records += [('/Odometry', NS + i * NS // 2, {'header': header(NS + i * NS // 2)}) for i in range(3)]
        r = analyze(records, info(**{'/lidar': CUSTOM, '/Odometry': 'nav_msgs/msg/Odometry'}))
        self.assertEqual(r['odometry_vs_scan'][0]['input_rate_hz'], 10.)
        self.assertEqual(r['odometry_vs_scan'][0]['output_rate_hz'], 2.)
        self.assertEqual(r['odometry_vs_scan'][0]['rate_ratio'], .2)
        self.assertIn('ODOMETRY_VS_SCAN', [x['code'] for x in r['findings']])

    def test_imu_gap_crossing_scan_boundary_marks_uncovered(self):
        stamps = [NS + i * 10_000_000 for i in [0, 1, 2, 3, 10, 11, 12]]
        c = self.coverage(stamps, base=NS + 50_000_000, offsets=[0, 60_000_000])
        self.assertEqual(c['covered'], 0)

    def test_reset_and_missing_imu_headers_leave_coverage_unknown(self):
        c = self.coverage([2 * NS, NS, NS + 10_000_000])
        self.assertEqual(c['unknown_scans'], 1)
        self.assertEqual(c['evaluable_scans'], 0)

    def test_pointcloud2_big_endian_and_row_padding(self):
        m = cloud(NS, [0., .125], endian=True)
        m.update(width=1, height=2, row_step=8, data=struct.pack('>f', 0.) + b'pad!' + struct.pack('>f', .125) + b'pad!')
        s = scan_summary(m, PC, contract())
        self.assertEqual(s['end_ns'], NS + 125_000_000)

    def test_malformed_cloud_fails_instead_of_silent_bounds(self):
        m = cloud(NS)
        m['row_step'] += 1
        with self.assertRaises(ValueError):
            scan_summary(m, PC)

    def test_nonfinite_point_times_leave_bounds_unknown(self):
        m = cloud(NS, [0., float('nan')])
        s = scan_summary(m, PC, contract())
        self.assertIsNone(s['start_ns'])
        self.assertEqual(s['fields'][0]['nonfinite_count'], 1)

    def test_sampling_preserves_all_headers_and_receipts(self):
        r = analyze([('/lidar', NS + i, custom(NS + i)) for i in range(5)], info(**{'/lidar': CUSTOM}), point_timing_sampled_every=2)
        t = r['topics'][0]
        self.assertEqual(t['count'], 5)
        self.assertEqual(t['header']['last_ns'], NS + 4)
        self.assertEqual(t['points']['scans_sampled'], 3)
        self.assertIsNone(t['points']['scan_bounds'][1]['start_ns'])

    def test_gap_examples_capped_without_losing_total(self):
        times = [NS]
        for _ in range(30):
            for d in [1, 1, 1, 10]:
                times.append(times[-1] + d * NS)
        r = timeline(times)
        self.assertEqual(len(r['gaps']), 20)
        self.assertEqual(r['gap_count'], 30)

    def test_drift_slope_and_nearest_imu_offset(self):
        records = [('/imu', (10 + i) * NS, imu((8 + i) * NS + i * 1000)) for i in range(5)]
        records += [('/lidar', 10 * NS, custom(8 * NS + 500))]
        r = analyze(records, info(**{'/imu': IMU, '/lidar': CUSTOM}))
        self.assertAlmostEqual(r['topics'][0]['header_minus_receipt_s']['drift_slope_s_per_s'], 1e-6, places=14)
        self.assertEqual(r['lidar_imu_header_offsets'][0]['nearest_header_offset_s']['median'], 500e-9)

    def test_explicit_clock_evidence_and_unknown_default(self):
        d = info(**{'/imu': IMU})
        d['/imu']['clocks'] = {'receipt': {'domain': 'postprocess-receipt', 'evidence': 'Synthetic replay provenance'}}
        t = analyze([('/imu', NS, imu(NS))], d)['topics'][0]
        self.assertEqual(t['clocks']['receipt']['domain'], 'postprocess-receipt')
        self.assertEqual(t['clock_domain'], 'unknown')

    def test_unsigned_point_offsets_do_not_wrap_when_decreasing(self):
        s = scan_summary(cloud(NS, [100, 0], 't', 6), PC, contract('t', 'ns'))
        self.assertEqual(s['backward_jumps_within_scan'], 1)
        self.assertEqual(s['duration_s'], 100e-9)


class ReaderCliTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-m', 'spatio.bagtiming', *map(str, args)], capture_output=True, text=True)

    def test_recorded_mid360_schema_preserves_exact_nanoseconds_without_fallback(self):
        self.assertEqual(
            hashlib.sha256(RECORDED_CUSTOM_SCHEMA.encode()).hexdigest(),
            'e1955256d7a20d899b742d06f929cba135a3393edd3a69cc3cb6dc762ba53230')
        base = 1_700_000_000 * NS + 17
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'recorded-layout.mcap'
            with bag.open('wb') as f:
                writer = Writer(f)
                schema = writer.register_msgdef(CUSTOM, RECORDED_CUSTOM_SCHEMA)
                writer.write_message('/livox/lidar', schema, custom(base, [5, 19]), log_time=base + 23)
                writer.finish()
            with patch('spatio.bagtiming.reader.manual_custommsg',
                       side_effect=AssertionError('The recorded schema must decode through mcap_ros2')):
                reader = BagReader(bag)
                records = list(reader.records())
            self.assertEqual(len(records), 1)
            topic, receipt, decoded = records[0]
            self.assertEqual((topic, receipt), ('/livox/lidar', base + 23))
            self.assertEqual(decoded.timebase, base)
            self.assertEqual([p.offset_time for p in decoded.points], [5, 19])
            scan = scan_summary(decoded, CUSTOM)
            self.assertEqual((scan['start_ns'], scan['end_ns']), (base + 5, base + 19))
            self.assertEqual(reader.manual_topics, set())

    def test_recorded_mid360_schema_cli_and_sampling(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'input').mkdir()
            bag = root / 'input/recorded-layout.mcap'
            with bag.open('wb') as f:
                writer = Writer(f)
                schema = writer.register_msgdef(CUSTOM, RECORDED_CUSTOM_SCHEMA)
                for i in range(3):
                    t = 10 * NS + i * NS
                    writer.write_message('/livox/lidar', schema, custom(t), log_time=t)
                writer.finish()
            proc = self.run_cli(bag, '--out', root / 'out')
            self.assertEqual(proc.returncode, 0, proc.stderr)
            result = json.loads((root / 'out/bagtiming.json').read_text())
            topic = result['topics'][0]
            self.assertEqual(topic['count'], 3)
            self.assertEqual(topic['points']['scan_duration_s']['median'], .1)
            self.assertEqual(topic['schema_definitions'], [RECORDED_CUSTOM_SCHEMA])
            self.assertIn('mcap_ros2', result['decoder_probes'][0]['reader'])
            reader = BagReader(bag, sample_every=2)
            sampled = analyze(reader.records(), reader.topic_info, point_timing_sampled_every=2)
            self.assertEqual(sampled['topics'][0]['header'], topic['header'])
            self.assertEqual(sampled['topics'][0]['points']['scans_sampled'], 2)

    def test_recorded_mid360_schema_manual_fallback_matches_decoder(self):
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'recorded-layout.mcap'
            with bag.open('wb') as f:
                writer = Writer(f)
                schema = writer.register_msgdef(CUSTOM, RECORDED_CUSTOM_SCHEMA)
                writer.write_message('/livox/lidar', schema, custom(10 * NS), log_time=10 * NS)
                writer.finish()
            reader = BagReader(bag)
            expected = analyze(reader.records(), reader.topic_info)
            with patch.object(DecoderFactory, 'decoder_for', side_effect=ValueError('forced decoder failure')):
                reader = BagReader(bag)
                actual = analyze(reader.records(), reader.topic_info)
            self.assertEqual(actual, expected)
            self.assertEqual(reader.manual_topics, {'/livox/lidar'})

    def test_cdr_or_decoder_roundtrip_small_synthetic_mcap(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'input').mkdir()
            bag = root / 'input/synthetic.mcap'
            write_bag(bag, custom_points=True)
            e = root / 'evidence.json'
            e.write_text(json.dumps({'timing_contracts': {'/points': contract()}}))
            proc = self.run_cli(bag, '--out', root / 'out', '--evidence', e)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            r = json.loads((root / 'out/bagtiming.json').read_text())
            self.assertEqual({x['name']: x['count'] for x in r['topics']}, {'/imu': 5, '/points': 2, '/livox/lidar': 3})
            self.assertEqual(r['metadata']['message_count'], 10)
            self.assertTrue(all(x['success'] for x in r['decoder_probes']))
            self.assertTrue(all('mcap_ros2' in x['reader'] for x in r['decoder_probes']))
            self.assertEqual(r['topics'][2]['points']['scan_duration_s']['median'], .1)
            self.assertIn('Compatibility table', (root / 'out/bagtiming.md').read_text())

    def test_cli_writes_json_and_md_and_exit_codes(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'input').mkdir()
            bag = root / 'input/synthetic.mcap'
            write_bag(bag)
            before = bag.read_bytes()
            proc = self.run_cli(bag.parent, '--out', root / 'out')
            self.assertEqual(proc.returncode, 0, proc.stderr)
            r = json.loads((root / 'out/bagtiming.json').read_text())
            self.assertEqual(r['schema'], 'spatio.bagtiming/1')
            self.assertIn('UNKNOWN_TIME_UNIT', [x['code'] for x in r['findings']])
            self.assertTrue((root / 'out/bagtiming.md').is_file())
            self.assertEqual(bag.read_bytes(), before)
            self.assertEqual(self.run_cli(root / 'missing', '--out', root / 'other').returncode, 2)
            broken = root / 'input/broken.mcap'
            broken.write_bytes(b'not a bag')
            self.assertEqual(self.run_cli(broken, '--out', root / 'other').returncode, 2)
            self.assertEqual(self.run_cli(bag, '--out', root / 'input').returncode, 2)

    def test_stored_order_preserves_receipt_backward_jumps(self):
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'synthetic.mcap'
            write_bag(bag, backwards=True)
            reader = BagReader(bag)
            r = analyze(reader.records(), reader.topic_info)
            t = next(x for x in r['topics'] if x['name'] == '/imu')
            self.assertEqual(t['receipt']['backward_jumps'], 1)
            self.assertEqual(t['header']['backward_jumps'], 0)

    def test_header_only_sampling_matches_full_decode(self):
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'synthetic.mcap'
            write_bag(bag, custom_points=True)
            full, sampled = BagReader(bag), BagReader(bag, sample_every=2)
            a = analyze(full.records(), full.topic_info)
            b = analyze(sampled.records(), sampled.topic_info, point_timing_sampled_every=2)
            for x, y in zip(a['topics'], b['topics']):
                self.assertEqual(x['header'], y['header'])
                self.assertEqual(x['receipt'], y['receipt'])
            self.assertEqual(b['topics'][2]['points']['scans_sampled'], 2)

    def test_slice_and_topic_selection_report_scope(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'input').mkdir()
            write_bag(root / 'input/synthetic.mcap')
            proc = self.run_cli(root / 'input', '--out', root / 'out', '--max-seconds', '.06', '--topics', '/imu')
            self.assertEqual(proc.returncode, 0, proc.stderr)
            r = json.loads((root / 'out/bagtiming.json').read_text())
            self.assertEqual(r['analysis']['message_count'], 2)
            self.assertEqual(r['metadata']['message_count'], 7)
            self.assertTrue(r['analysis']['stopped_at_slice_boundary'])

    def test_manual_cdr_hand_built_buffer_both_endiannesses(self):
        # Independently hand-pack CDR, including header string and uint64 alignment.
        for endian, enc in [('<', b'\x00\x01\x00\x00'), ('>', b'\x00\x00\x00\x00')]:
            data = bytearray(enc)
            def put(fmt, alignment, *values):
                data.extend(b'\0' * ((-(len(data) - 4)) % alignment))
                data.extend(struct.pack(endian + fmt, *values))
            put('iII', 4, 10, 23, 4)
            data.extend(b'abc\0')
            put('Q', 8, 10 * NS + 23)
            put('IBBBB', 4, 2, 1, 0, 0, 0)
            put('I', 4, 2)
            for offset in [0, 125000000]:
                put('IfffBBB', 4, offset, 1., 2., 3., 4, 0, 1)
            decoded = manual_custommsg(bytes(data))
            self.assertEqual(decoded['timebase'], 10 * NS + 23)
            self.assertEqual(decoded['points'][1]['offset_time'], 125000000)
            self.assertEqual(decoded['header']['frame_id'], 'abc')
            self.assertEqual(scan_summary(decoded, CUSTOM)['duration_s'], .125)
            with self.assertRaises(ValueError):
                manual_custommsg(bytes(data[:-1]))

    def test_manual_custommsg_matches_embedded_schema_decoder(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'synthetic.mcap'
            write_bag(path, custom_points=True)
            with path.open('rb') as f:
                r = make_reader(f)
                for schema, ch, message in r.iter_messages(topics=['/livox/lidar'], log_time_order=False):
                    decoded = DecoderFactory().decoder_for(ch.message_encoding, schema)(message.data)
                    manual = manual_custommsg(message.data)
                    self.assertEqual(scan_summary(decoded, CUSTOM), scan_summary(manual, CUSTOM))

class BoundaryTests(unittest.TestCase):
    def test_recorded_schema_normalization_preserves_strict_layout(self):
        from mcap.records import Schema
        from spatio.bagtiming.reader import custom_schema_supported, schema_sections
        def supported(text):
            return custom_schema_supported(Schema(id=1, name=CUSTOM, encoding='ros2msg', data=text.encode()))
        self.assertTrue(supported(RECORDED_CUSTOM_SCHEMA))
        self.assertTrue(supported(RECORDED_CUSTOM_SCHEMA.replace('uint8  lidar_id', 'uint8\t lidar_id')))
        self.assertEqual(schema_sections(RECORDED_CUSTOM_SCHEMA, CUSTOM)[CUSTOM],
                         ['std_msgs/Header header', 'uint64 timebase', 'uint32 point_num',
                          'uint8 lidar_id', 'uint8[3] rsvd', 'CustomPoint[] points'])
        for before, after in [('uint64 timebase', 'uint32 timebase'),
                              ('uint32 offset_time', 'float32 offset_time'),
                              ('uint8[3]', 'uint8[4]'),
                              ('uint8  lidar_id', 'uint8  device_id'),
                              ('float32 x', 'float32 other'),
                              ('CustomPoint[] points', 'CustomPoint[2] points')]:
            with self.subTest(after=after):
                self.assertFalse(supported(RECORDED_CUSTOM_SCHEMA.replace(before, after)))
        reordered = RECORDED_CUSTOM_SCHEMA.replace('float32 x', 'float32 temporary')
        reordered = reordered.replace('float32 y', 'float32 x').replace('float32 temporary', 'float32 y')
        self.assertFalse(supported(reordered))

    def test_custom_schema_must_include_standard_header_contract(self):
        from mcap.records import Schema
        from spatio.bagtiming.reader import custom_schema_supported
        self.assertTrue(custom_schema_supported(Schema(id=1, name=CUSTOM, encoding='ros2msg', data=CUSTOM_SCHEMA.encode())))
        bad = CUSTOM_SCHEMA.replace('int32 sec\nuint32 nanosec', 'uint32 nanosec\nint32 sec')
        self.assertFalse(custom_schema_supported(Schema(id=1, name=CUSTOM, encoding='ros2msg', data=bad.encode())))

    def test_missing_imu_topic_does_not_silently_imply_coverage(self):
        r = analyze([('/lidar', NS, custom(NS))], info(**{'/lidar': CUSTOM}))
        self.assertIn('NO_IMU_TOPIC', [f['code'] for f in r['findings']])
        self.assertEqual(r['imu_coverage'], [])

    def test_nested_tf_frames_without_fabricated_header(self):
        r = analyze([('/tf', NS, {'transforms': [{'header': header(NS, 'map')} ]})], info(**{'/tf': 'tf2_msgs/msg/TFMessage'}))
        t = r['topics'][0]
        self.assertEqual(t['frame_ids'], ['map'])
        self.assertIsNone(t['header']['first_ns'])
        self.assertIsNone(t['points'])

    def test_invalid_timing_contract_cannot_establish_bounds(self):
        with self.assertRaises(ValueError):
            scan_summary(cloud(NS), PC, {'field': 'time', 'unit': 's', 'origin': 'header'})
        with self.assertRaises(ValueError):
            scan_summary(cloud(NS), PC, contract('nonexistent'))

    def test_small_backward_point_and_timebase_jump_preserve_evidence(self):
        r = analyze([('/lidar', 2 * NS, custom(2 * NS)), ('/lidar', 3 * NS, custom(NS // 2))], info(**{'/lidar': CUSTOM}))
        p = r['topics'][0]['points']
        self.assertEqual(p['timebase']['backward_jumps'], 1)
        self.assertEqual(len(p['timebase']['resets']), 1)
        self.assertEqual(p['scan_start']['first_ns'], 2 * NS)
        self.assertIn('POINT_TIME_DISCONTINUITY', [f['code'] for f in r['findings']])

    def test_point_epoch_threshold_crossing_is_reset_even_below_one_second(self):
        s = scan_summary(custom(10**18 - 100, [200, 0]), CUSTOM)
        self.assertEqual(s['resets_within_scan'], 1)
        self.assertEqual(s['backward_jumps_within_scan'], 1)


if __name__ == '__main__':
    unittest.main()
