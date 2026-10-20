"""IMU stream characterizer for Spatio milestone 1.

The physics lives in analyze.py and is pure numpy. The bag reader is separate.
Run ``python3 -m spatio.imucheck --help``.
"""

from spatio.imucheck.analyze import characterize

__all__ = ["characterize"]
