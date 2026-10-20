# JT128 adapter characterization

The two Python files in `vendor/` are unchanged staged copies. See
`vendor/PROVENANCE.md`: this adapter is **not verified to be the version that
produced the 20260826T221936Z bag**. This package characterizes behavior; it does
not fix the adapter or run ROS/FAST-LIO.

From the assigned worktree:

```sh
cd ~/Developer/spatio-x2-jt128-adapter
nice -n 10 python3 -m unittest discover -s spatio/jt128adapter/tests -t . -v
```

The system Python 3.9 environment supplies numpy, mcap and mcap_ros2. No new
packages are required. Synthetic MCAP fixtures are generated inside `_out/`
and cleaned up by the tests. The upstream PCAP-dependent test is the only
skip; its input was not copied. The two independent upstream tests execute
unchanged. An empty test-harness `jt128_decode` module satisfies their unused
eager import; it implements no decoder. The adapter itself imports without
ROS and without stubs. ROS `main()` is checked statically, not executed.

Run a bounded slice before a full pass (remove `--max-seconds 1` for full):

```sh
/usr/bin/lockf -k -t 1800 ~/Developer/lc-tasks/.spatio-bigio.lock \
  nice -n 10 python3 -m spatio.jt128adapter.bagcheck \
  ~/Developer/spatio-data/recordings/20260826T221936Z_JT128_room_walk \
  --out ~/Developer/spatio-data/derived/x2-jt128-adapter/ --max-seconds 1
```

If the external output directory is refused, use
`--out spatio/jt128adapter/_out/real-data`. The **same shared lock** is still
required. Do not replace it with a private lock or run unlocked. This session's
shared-lock and Git-index writes were denied; see the findings in `_out/`.

`adaptercheck.json` includes the vendor hash, reader versions, input file sizes,
whole-bag topic inventory from MCAP summaries, decoded message counts, scope,
per-scan data, aggregate distributions, IMU matching and eight verdicts.
`adaptercheck.md` summarizes the same evidence. Field names with `_s` use the
vendored seconds interpretation; that naming does not prove the bag's units.
Receipt time is never labelled acquisition or firing time.

Every "consistent with vendored source" verdict requires an unrestricted full
pass and decoded counts equal to the whole-bag inventory for all three analyzed
topics (`/slam/points`, `/lidar_imu`, `/slam/imu`). The report records this as
`scope.whole_bag_covered`. Any `--max-seconds` window, even one containing all
messages, leaves positive consistency "not determinable from this bag".
Contradictions observed in a subset still produce "inconsistent (evidence)".

The reader explicitly uses `NonSeekingReader` in physical file order. Payload
memory scales with MCAP chunk size and the current decoded cloud, rather than
all clouds in the bag; compact per-scan/IMU records still grow with the recording.
A time limit selects a half-open window relative to the earliest MCAP receipt
timestamp. It limits decoded messages, not bytes read: the sequential reader
scans the whole file so out-of-order receipt timestamps are not lost. The current
implementation requires MCAP summaries/statistics. Matching consumes each IMU sample once: all exact
integer-nanosecond matches first, then chronological greedy nearest within
1 ms. It reports both topic match rates, unmatched timestamps and their positions
relative to the other topic's range. It cannot identify recorder/subscriber
loss from a count difference alone. Ratios exclude exactly zero raw components;
residuals include them. Signed-axis permutations are compared by RMS residual,
with uniqueness reported. Numerical consistency uses a 1e-12 absolute residual
bound and requires complete pairing of both inspected IMU streams plus at least
one finite nonzero raw component on every axis. Zero-only axes or incomplete
pairing produce "not determinable from this bag" unless a matched contradiction
already establishes inconsistency. This is a conservative evidence rule, not
a statistical confidence or calibration test; physical units remain unknown.

Raw `/lidar_points` is absent from the supplied recording's metadata. Without
it, adapted clouds cannot establish input stride, ring preservation, time
origin, firing-time units or the source of their header stamps. Zero-starting
times or plausible durations alone are insufficient. Synthetic tests establish
these behaviors only for the preserved source and controlled input pattern.
