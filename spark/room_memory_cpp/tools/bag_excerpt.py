"""Copies the first messages of some topics from a rosbag2 mcap into a new mcap (zstd chunks),
byte for byte, with their schemas, channels and log times:

    python bag_excerpt.py <bag dir or .mcap> <out.mcap> --topic /livox/lidar --count 8 --from-s 40

This is how tests/fixtures/static5m-mid360-excerpt.mcap was made from the static recording.
"""

import argparse

from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap.writer import CompressionType, Writer

from bag_stream import segments


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag")
    parser.add_argument("out")
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--count", type=int, required=True, help="messages per topic")
    parser.add_argument("--from-s", type=float, default=0.0, help="skip messages before this time from the start")
    args = parser.parse_args(argv)
    files = segments(args.bag)
    with open(args.out, "wb") as handle:
        writer = Writer(handle, compression=CompressionType.ZSTD)
        writer.start(profile="ros2", library="room_memory bag_excerpt")
        schemas, channels, ids, left = {}, {}, {}, {topic: args.count for topic in args.topic}
        first = None
        for file in files:
            if not any(left.values()):
                break
            with open(file, "rb") as bag:
                for record in StreamReader(bag).records:
                    if isinstance(record, Schema):
                        schemas[record.id] = record
                    elif isinstance(record, Channel):
                        channels[record.id] = record
                    elif isinstance(record, Message):
                        first = record.log_time if first is None else first
                        channel = channels[record.channel_id]
                        if left.get(channel.topic, 0) == 0 or (record.log_time - first) / 1e9 < args.from_s:
                            continue
                        if channel.id not in ids:
                            schema = schemas[channel.schema_id]
                            schema_id = writer.register_schema(schema.name, schema.encoding, schema.data)
                            ids[channel.id] = writer.register_channel(channel.topic, channel.message_encoding, schema_id,
                                                                      channel.metadata)
                        writer.add_message(ids[channel.id], record.log_time, record.data, record.publish_time,
                                           record.sequence)
                        left[channel.topic] -= 1
                        if not any(left.values()):
                            break
        writer.finish()


if __name__ == "__main__":
    main()
