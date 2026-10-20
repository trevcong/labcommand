"""MCAP IO boundary. Stored message order is never timestamp-sorted."""
from __future__ import annotations

from importlib.metadata import version
from pathlib import Path
import re
import struct

from mcap.reader import make_reader, NonSeekingReader
from mcap_ros2.decoder import DecoderFactory


class InputError(ValueError):
    pass


def schema_sections(text, root_type):
    sections = {root_type: []}
    current = root_type
    for raw in text.splitlines():
        line = raw.split('#', 1)[0].strip()
        if line.startswith('MSG: '):
            current = line[5:].strip()
            sections.setdefault(current, [])
        elif line and not line.startswith('='):
            sections[current].append(line)
    return sections


def schema_fields(text, root_type):
    return schema_sections(text, root_type)[root_type]


class CDR:
    """Exact CDR1 primitives needed for Header and Livox CustomMsg only."""
    def __init__(self, data):
        if len(data) < 4 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise InputError('Only CDR1 big/little endian encapsulations supported by minimal parser')
        self.data, self.pos = data, 4
        self.endian = '<' if data[1] == 1 else '>'

    def read(self, fmt, alignment):
        self.pos += (-(self.pos - 4)) % alignment
        size = struct.calcsize(fmt)
        if self.pos + size > len(self.data):
            raise InputError('Truncated CDR')
        val = struct.unpack_from(self.endian + fmt, self.data, self.pos)
        self.pos += size
        return val[0] if len(val) == 1 else val

    def string(self):
        n = self.read('I', 4)
        if n < 1 or self.pos + n > len(self.data) or self.data[self.pos + n - 1] != 0:
            raise InputError('Invalid CDR string')
        out = self.data[self.pos:self.pos + n - 1].decode('utf8')
        self.pos += n
        return out

    def header(self):
        sec, nanosec = self.read('iI', 4)
        return {'stamp': {'sec': sec, 'nanosec': nanosec}, 'frame_id': self.string()}


def header_only(data):
    return {'header': CDR(data).header(), '_point_timing_skipped': True}


def manual_custommsg(data):
    c = CDR(data)
    h, base = c.header(), c.read('Q', 8)
    point_num = c.read('I', 4)
    lidar = c.read('B', 1)
    reserved = c.read('3B', 1)
    n = c.read('I', 4)
    if n > len(data) // 19:
        raise InputError('Invalid CustomMsg array length')
    points = []
    for _ in range(n):
        offset = c.read('I', 4)
        c.read('fff', 4)
        c.read('BBB', 1)
        points.append({'offset_time': offset})
    if c.pos != len(data):
        raise InputError('Unexpected trailing CustomMsg data')
    return {'header': h, 'timebase': base, 'point_num': point_num,
            'lidar_id': lidar, 'rsvd': reserved, 'points': points}


def header_schema_supported(schema):
    sections = schema_sections(schema.data.decode(), schema.name)
    header = next((v for k, v in sections.items() if k in ('std_msgs/Header', 'std_msgs/msg/Header')), [])
    stamp = next((v for k, v in sections.items() if k in ('builtin_interfaces/Time', 'builtin_interfaces/msg/Time')), [])
    return (header in (['builtin_interfaces/Time stamp', 'string frame_id'],
                       ['builtin_interfaces/msg/Time stamp', 'string frame_id'])
            and stamp == ['int32 sec', 'uint32 nanosec'])


def custom_schema_supported(schema):
    sections = schema_sections(schema.data.decode(), schema.name)
    root = sections[schema.name]
    points = next((v for k, v in sections.items() if k.endswith('/CustomPoint')), [])
    normalized = [re.sub(r'\b(?:livox_ros_driver2/(?:msg/)?)?CustomPoint\[\]', 'CustomPoint[]', x) for x in root]
    normalized = [x.replace('std_msgs/msg/Header', 'std_msgs/Header') for x in normalized]
    return (header_schema_supported(schema) and normalized == ['std_msgs/Header header', 'uint64 timebase', 'uint32 point_num',
                           'uint8 lidar_id', 'uint8[3] rsvd', 'CustomPoint[] points']
            and points == ['uint32 offset_time', 'float32 x', 'float32 y', 'float32 z',
                           'uint8 reflectivity', 'uint8 tag', 'uint8 line'])


def resolve_bags(path):
    path = Path(path).expanduser().resolve()
    if path.is_file() and path.suffix.lower() == '.mcap':
        return [path]
    if path.is_dir():
        files = list(path.glob('*.mcap'))
        if not files:
            files = list((path / 'bag').glob('*.mcap'))
        # bag_2 must precede bag_10. Storage order across segments, no time sort.
        files.sort(key=lambda p: [int(x) if x.isdigit() else x for x in re.split('(\d+)', p.name)])
        if files:
            return files
    raise InputError('Expected a readable .mcap file or directory containing MCAP segments')


class BagReader:
    def __init__(self, path, max_seconds=None, topics=None, sample_every=1, clocks=None):
        self.files = resolve_bags(path)
        self.max_seconds, self.selected, self.sample_every = max_seconds, set(topics) if topics else None, sample_every
        self.topic_info = {}
        self.metadata = {'duration_s': None, 'starting_time_ns': None, 'message_count': None}
        self.reader_name = 'mcap_ros2 ' + version('mcap-ros2-support')
        self.decoder_probes = []
        self.manual_topics = set()
        self.clocks = clocks or {}
        self.inventory = []
        self.bytes = sum(p.stat().st_size for p in self.files)
        self.read_count = 0
        self.partial = False
        starts, ends, counts = [], [], []
        for p in self.files:
            with p.open('rb') as f:
                summary = make_reader(f, validate_crcs=True).get_summary()
                if summary and summary.statistics:
                    st = summary.statistics
                    starts.append(st.message_start_time)
                    ends.append(st.message_end_time)
                    counts.append(st.message_count)
                    for cid, ch in summary.channels.items():
                        schema = summary.schemas.get(ch.schema_id)
                        self.inventory.append({'segment': p.name, 'topic': ch.topic,
                                               'type': schema.name if schema else None,
                                               'count': st.channel_message_counts.get(cid)})
        if len(counts) == len(self.files):
            self.metadata = {'duration_s': (max(ends) - min(starts)) / 1e9,
                             'starting_time_ns': min(starts), 'message_count': sum(counts)}
        self.metadata['scope'] = 'full bag MCAP summary, not the selected slice; null if unavailable'
        self.metadata['segments'] = [str(p) for p in self.files]
        self.metadata['inventory'] = self.inventory

    def records(self):
        seen, probes = {}, set()
        origin = self.metadata['starting_time_ns']
        for p in self.files:
            factory = DecoderFactory()  # schema IDs are local to a segment
            with p.open('rb') as f:
                reader = NonSeekingReader(f, validate_crcs=True)
                for schema, channel, message in reader.iter_messages(log_time_order=False):
                    if origin is None:
                        origin = message.log_time
                    if self.max_seconds is not None and message.log_time - origin > self.max_seconds * 1e9:
                        self.partial = True
                        return
                    if self.selected is not None and channel.topic not in self.selected:
                        continue
                    if schema is None or schema.encoding != 'ros2msg' or channel.message_encoding != 'cdr':
                        raise InputError('Unsupported schema/encoding for ' + channel.topic)
                    text = schema.data.decode('utf8')
                    if schema.name == 'livox_ros_driver2/msg/CustomMsg' and not custom_schema_supported(schema):
                        raise InputError('Livox schema differs from the supported source-backed timing contract')
                    info = self.topic_info.setdefault(channel.topic, {'type': schema.name, 'fields': schema_fields(text, schema.name),
                                                       'schema_definitions': [], 'clocks': self.clocks.get(channel.topic, {})})
                    if info['type'] != schema.name:
                        raise InputError('Topic changes message type: ' + channel.topic)
                    if text not in info['schema_definitions']:
                        info['schema_definitions'].append(text)
                    idx = seen.get(channel.topic, 0)
                    seen[channel.topic] = idx + 1
                    lidar = schema.name in ('livox_ros_driver2/msg/CustomMsg', 'sensor_msgs/msg/PointCloud2')
                    can_header = (info['fields'] and info['fields'][0] in ('std_msgs/Header header', 'std_msgs/msg/Header header')
                                  and header_schema_supported(schema))
                    key = (str(p), channel.topic)
                    if lidar and idx % self.sample_every and can_header:
                        decoded = header_only(message.data)
                    else:
                        try:
                            decoder = factory.decoder_for(channel.message_encoding, schema)
                            if decoder is None:
                                raise InputError('No mcap_ros2 decoder')
                            decoded = decoder(message.data)
                            if key not in probes:
                                self.decoder_probes.append({'file': str(p), 'topic': channel.topic, 'type': schema.name,
                                                            'bytes': len(message.data), 'reader': self.reader_name, 'success': True})
                                probes.add(key)
                        except Exception as exc:
                            if schema.name != 'livox_ros_driver2/msg/CustomMsg' or not custom_schema_supported(schema):
                                raise InputError('Cannot decode {}: {}'.format(channel.topic, exc)) from exc
                            decoded = manual_custommsg(message.data)
                            self.manual_topics.add(channel.topic)
                            if key not in probes:
                                self.decoder_probes.append({'file': str(p), 'topic': channel.topic, 'type': schema.name,
                                                            'reader': 'manual-cdr', 'success': True, 'decoder_error': str(exc)})
                                probes.add(key)
                    self.read_count += 1
                    yield channel.topic, message.log_time, decoded

    def description(self):
        suffix = ' + manual-cdr CustomMsg fallback' if self.manual_topics else ''
        suffix += ' + CDR1 header-only unsampled lidar' if self.sample_every > 1 else ''
        return self.reader_name + suffix
