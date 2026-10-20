"""Synthetic, voxel-resolution clearance evidence; not a compliance certificate."""

from .model import load_scans, load_zones, parse_scans, parse_zones
from .grid import EvidenceGrid
from .classify import classify

__all__ = ["EvidenceGrid", "classify", "load_scans", "load_zones",
           "parse_scans", "parse_zones"]
