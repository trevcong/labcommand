# Vendor provenance

not verified to be the version that produced the 20260826T221936Z bag

Primary source: /Users/tconf/Developer/lc-tasks/staging/jt128-adapter/. Files copied byte for byte with metadata preserved. SHA256SUMS supplied by the lead agrees.

| File | Bytes | Source mtime (UTC) | SHA256 |
|---|---:|---|---|
| jt128_slam_adapter.py | 9255 | 2026-08-26T23:23:14.799647+00:00 | 1eee0c0173e0fe622f2964e378426de0be9fec29678041564171a492e73c3df8 |
| test_jt128_slam_adapter.py | 4166 | 2026-08-26T23:18:20.978159+00:00 | 6ae8bf50835379384cbd22b01b9c921e6e3e03c832dd4cda87b964e833be6311 |

Original paths: /Users/tconf/Desktop/researchfindings/scripts/jt128_slam_adapter.py and /Users/tconf/Desktop/researchfindings/tests/test_jt128_slam_adapter.py. Desktop files were read only to confirm hashes; neither file was executed there.
Desktop confirmation: /Users/tconf/Desktop/researchfindings/scripts/jt128_slam_adapter.py: 9255 bytes, SHA256 1eee0c0173e0fe622f2964e378426de0be9fec29678041564171a492e73c3df8.
Desktop confirmation: /Users/tconf/Desktop/researchfindings/tests/test_jt128_slam_adapter.py: 4166 bytes, SHA256 6ae8bf50835379384cbd22b01b9c921e6e3e03c832dd4cda87b964e833be6311.

Spark-copy adapter: /Users/tconf/Developer/spatio-data/spark-src/launchers/jt128_slam_adapter.py (0 bytes). No Spark access was made. Bag postprocess.yaml states 2026-08-27T01:42:36Z; mtime/hash agreement does not establish execution provenance.

Pure helpers import without ROS or stubs. ROS main() is not executed. Upstream tests import an unavailable jt128_decode module eagerly; harness supplies an empty module solely to load the two independent tests. The real-PCAP test alone is explicitly skipped because its input is absent. Vendor files are never edited.
