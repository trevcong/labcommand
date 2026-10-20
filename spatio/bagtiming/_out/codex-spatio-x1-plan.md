# x1 bag timing implementation plan

Read both shared plans fully. Verified branch spatio-x1-bagtiming at d675f8a, clean tree; COPY_DONE 0.

Modules: reader.py streams MCAP in stored order using embedded schemas and mcap_ros2; core.py consumes (topic, receipt_ns, decoded message) records without file IO; report.py renders evidence and compatibility tables; __main__.py handles CLI and artifacts; tests/ uses unittest and tiny generated MCAP.

Reader decision first: probe several CustomMsg/Imu/PointCloud2 messages under the shared lock. Use embedded-schema decoder if successful; otherwise implement and hand-buffer-test a minimal CDR fallback. Preserve file order to detect backward receipt/header timestamps. Retain only timestamp arrays and per-scan summaries, never point clouds.

Time semantics: integers for ns, subtract before converting to seconds. Never infer PointCloud2 units or relative/absolute origin from names, datatypes or magnitude alone. Accept explicit evidence contracts; otherwise report raw field ranges and unknown unit/bounds/coverage. Livox message definitions establish names; copied driver and FAST-LIO source establish ns. Classify clocks per time base with evidence, default unknown. Coverage uses bracketing IMU samples and intersecting gap intervals, and refuses to establish synchronization solely from overlap.

Tests: all 17 named task tests, plus malformed cloud layout, big endian/padded rows, zeros/reset/gap edge cases, unknown clock/unit, source ordering, and bounded output as necessary. CLI roundtrip uses actual mcap_ros2 writer and subprocess. No skips. No Swift changes or tests.

Real runs: commit once tests pass; attempt authorized output and shared lock paths without sandbox workaround. Time 30-second slice of each sequentially under lock; extrapolate. If needed choose k for point sampling while retaining every header/receipt. Full sequential runs under same lock; reports explicitly state exclusions and sample counts. If lock/output writes refused, follow task fallback to owned _out; do not bypass shared locking. Record measured numbers, all warnings/errors, compatibility table and limits in findings; second local commit ends [skip ci].

Resolved decisions: namespace spatio needs no init; use stdlib plus installed numpy/mcap/mcap_ros2, no installs. Multiple MCAP segments supported in filename order, documented. Unknowns block compatibility claims, never lead to timestamp repair. No Spark access or other-workstream imports.
