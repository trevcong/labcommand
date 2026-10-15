"""Puts the package and the test helpers on ``sys.path`` for plain ``pytest`` runs."""

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
