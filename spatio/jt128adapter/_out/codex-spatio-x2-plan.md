# X2 implementation plan

Read both shared plans fully; verified assigned branch and clean tree at start.

Modules: vendor/ contains byte-identical staged adapter and upstream tests, with PROVENANCE.md. bagcheck.py streams system mcap/mcap_ros2 decoded messages and uses numpy views of each cloud. Tests use unittest, synthetic Hesai payloads and a tiny generated MCAP. No other spatio imports, no ROS runtime, no Swift changes.

Data flow: recording bag files -> decoded /slam/points and raw/adapted IMU -> per-scan statistics, integer-nanosecond one-to-one stamp pairing (all exact matches before nearest within 1 ms), per-axis ratios and permutation residuals -> adaptercheck.json and adaptercheck.md. Retain compact IMU values and per-scan summaries, never whole clouds. Use physical file order for streaming; max-seconds is a bag-receipt-time window. Full real pass under required lock, after initial test commit.

Tests: all 16 named done criteria, plus boundary and report-honesty cases. Upstream module loaded by file path; only missing-PCAP test skipped. Its unused decoder import will be a harness-only unavailable-module stub, never a fake decoder. ROS main() accel/header/layout paths inspected statically using AST and corroborated by bag; no ROS callback execution claimed. Vendor imports must never import rclpy.

Questions resolved: ring retention is tested separately from stride-induced coverage loss; point relative-time origin is not necessarily header stamp or scan acquisition start. Float error measured separately for float64 epoch quantization and float32 relative conversion. Raw points absent means stride, firing timing, point-header source, and input ring mapping cannot be proven by bag. IMU count imbalance will be reported by unmatched stamp location/duplicates; transport cause remains unknown unless evidence identifies it. Acceleration normalization checked against copied FAST-LIO source. Changes remain inside owned folder.

Outputs: requested external derived/findings/plan paths where permitted; otherwise specified _out fallback. Commit tests before full real-data read, then commit measured findings, messages ending [skip ci].
