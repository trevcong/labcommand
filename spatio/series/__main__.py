"""CLI entry point: ``python3 -m spatio.series``."""

from __future__ import annotations

import sys

from spatio.series.cli import main

if __name__ == "__main__":
    sys.exit(main())
