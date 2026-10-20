"""Collect upstream tests through their existing offline harness exactly once.

tests/test_characterization.py loads the immutable vendor test module with
scoped import stubs, runs both synthetic tests, and reports the unavailable
real-PCAP test as skipped. Direct collection bypasses that harness and imports
the absent decoder. Do not stub modules globally or modify the vendor copy.
"""

collect_ignore = ["vendor/test_jt128_slam_adapter.py"]
