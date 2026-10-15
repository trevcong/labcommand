# Room memory benchmark

Status: **complete**

| Metric | Value |
| --- | --- |
| Backend | python |
| Replay / wall seconds | 60 / 73.023 |
| Ticks / frames / points per frame | 600 / 1200 / 80000 |
| Tick p50, ms | 0.008 |
| Tick p95, ms | 998.984 |
| Tick p99, ms | 1393.130 |
| Tick max, ms | 1804.447 |
| Hits / eligible sensor targets | 6 / 6 |
| Hit rate | 1.0 |
| False alarms / hour | 0.0 |
| False alarms / exposure seconds | 0 / 55 |
| Censored sensor targets | 0 |
| Harness RSS start / end / sampled peak, MiB | 27.50 / 436.41 / 1782.02 |
| Harness RSS growth, MiB | 408.91 |
| Harness RSS slope, bytes/replay second | -7956655.542857125 |
| Detection latency p50, replay seconds | 1.0 |
| Detection latency p95, replay seconds | 1.0 |
| Detection latency p99, replay seconds | 1.0 |
| Detection latency max, replay seconds | 1.0 |

Tick timings include both sensors and window closure/status when due; generation and RSS sampling are excluded.
Replay is unpaced. Candidate hits are per sensor, with a 10-second deadline. See README for matching and caveats.
RSS is sampled at replay boundaries; transient peaks between samples are not measured.
