"""Throttled warning text for rejected lidar frames. No ROS or numpy imports.

One line per sensor per ``INTERVAL_S`` seconds. The caller logs it. The helper sees the
exception type name and the message string, never the exception.
"""

import time

INTERVAL_S = 10.0


class RejectionLog:
    """One warning line per sensor per interval, with a suppressed count.

    ``clock`` is a zero-argument callable that returns seconds as a float. The default is
    ``time.monotonic``, so a wall-clock step does not open or close a window.
    """

    def __init__(self, interval_s=INTERVAL_S, clock=time.monotonic):
        self._interval_s = interval_s
        self._clock = clock
        self._last_emit = {}
        self._suppressed = {}

    def observe(self, sensor, type_name, message):
        """Return the warning line, or None when this rejection is suppressed."""
        now = self._clock()
        last = self._last_emit.get(sensor)
        if last is not None and now - last < self._interval_s:
            self._suppressed[sensor] = self._suppressed.get(sensor, 0) + 1
            return None
        count = self._suppressed.get(sensor, 0)
        self._last_emit[sensor] = now
        self._suppressed[sensor] = 0
        line = f"{sensor} rejected frame: {type_name}: {message}"
        if count > 0:
            line = f"{line} ({count} suppressed)"
        return line
