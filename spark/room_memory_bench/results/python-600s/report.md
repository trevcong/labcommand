# Room memory benchmark

Status: **complete**

| Metric | Value |
| --- | --- |
| Backend | python |
| Replay / wall seconds | 600 / 666.232 |
| Ticks / frames / points per frame | 6000 / 12000 / 80000 |
| Tick p50, ms | 0.006 |
| Tick p95, ms | 848.359 |
| Tick p99, ms | 1205.735 |
| Tick max, ms | 1648.171 |
| Hits / eligible sensor targets | 60 / 60 |
| Hit rate | 1.0 |
| False alarms / hour | 0.0 |
| False alarms / exposure seconds | 0 / 595 |
| Censored sensor targets | 0 |
| Harness RSS start / end / sampled peak, MiB | 27.38 / 467.56 / 1727.77 |
| Harness RSS growth, MiB | 440.19 |
| Harness RSS slope, bytes/replay second | -123799.53142252649 |
| Detection latency p50, replay seconds | 1.0 |
| Detection latency p95, replay seconds | 1.0 |
| Detection latency p99, replay seconds | 1.0 |
| Detection latency max, replay seconds | 1.0 |

Tick timings include both sensors and window closure/status when due; generation and RSS sampling are excluded.
Replay is unpaced. Candidate hits are per sensor, with a 10-second deadline. See README for matching and caveats.
RSS is sampled at replay boundaries; transient peaks between samples are not measured.
