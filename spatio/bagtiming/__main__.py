"""python3 -m spatio.bagtiming INPUT --out DIR"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

from .core import analyze
from .reader import BagReader, InputError
from .report import markdown


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read-only MCAP receipt/header/point timing inspector; never corrects timestamps.')
    parser.add_argument('bag')
    parser.add_argument('--out', required=True)
    parser.add_argument('--max-seconds', type=float)
    parser.add_argument('--topics', nargs='+')
    parser.add_argument('--gap-factor', type=float, default=3.0)
    parser.add_argument('--point-timing-sampled-every', type=int, default=1)
    parser.add_argument('--evidence', help='Optional JSON with timing_contracts and clocks, each supported by evidence text')
    args = parser.parse_args(argv)
    if args.max_seconds is not None and (not math.isfinite(args.max_seconds) or args.max_seconds <= 0):
        parser.error('--max-seconds must be finite and positive')
    if not math.isfinite(args.gap_factor) or args.gap_factor <= 0 or args.point_timing_sampled_every < 1:
        parser.error('gap-factor and point-timing-sampled-every must be positive')
    started = time.monotonic()
    try:
        evidence = json.loads(Path(args.evidence).read_text()) if args.evidence else {}
        reader = BagReader(args.bag, args.max_seconds, args.topics, args.point_timing_sampled_every, evidence.get('clocks'))
        # Prevent overwriting inputs or writing inside a recording directory.
        out = Path(args.out).expanduser().resolve()
        for path in reader.files:
            if out == path.parent or path.parent in out.parents or out in (path,):
                raise InputError('Output must be outside the input MCAP directory')
        result = analyze(reader.records(), reader.topic_info, args.gap_factor,
                         evidence.get('timing_contracts'), args.point_timing_sampled_every)
        result.update(bag=str(Path(args.bag).expanduser().resolve()), bag_bytes=reader.bytes,
                      reader=reader.description(), metadata=reader.metadata,
                      decoder_probes=reader.decoder_probes,
                      evidence_contract=evidence,
                      analysis={'message_count': reader.read_count, 'max_seconds': args.max_seconds,
                                'topics_filter': args.topics, 'stopped_at_slice_boundary': reader.partial,
                                'elapsed_wall_s': time.monotonic() - started})
        if args.max_seconds is not None:
            result['findings'].append({'severity': 'warn', 'code': 'PARTIAL_INSPECTION',
                                      'text': 'Requested {} s receipt slice; {} messages inspected. Full-bag discontinuities not evaluated.'.format(args.max_seconds, reader.read_count)})
        if args.topics:
            missing = set(args.topics) - {t['name'] for t in result['topics']}
            if missing:
                result['findings'].append({'severity': 'warn', 'code': 'TOPICS_NOT_OBSERVED', 'text': 'Not observed in selected interval: ' + ', '.join(sorted(missing))})
        if not result['topics']:
            result['findings'].append({'severity': 'warn', 'code': 'NO_MESSAGES', 'text': 'No selected messages inspected.'})
        serialized = json.dumps(result, indent=2, allow_nan=False) + '\n'
        md = markdown(result)
    except Exception as exc:
        print('bagtiming: input unreadable or invalid: {}'.format(exc), file=sys.stderr)
        return 2
    try:
        out.mkdir(parents=True, exist_ok=True)
        (out / 'bagtiming.json').write_text(serialized)
        (out / 'bagtiming.md').write_text(md)
    except OSError as exc:
        print('bagtiming: cannot write output: {}'.format(exc), file=sys.stderr)
        return 1
    print('Inspected {} messages in {:.3f} s; {} findings; {}'.format(reader.read_count, result['analysis']['elapsed_wall_s'], len(result['findings']), out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
