"""Defines the top-level API for the IMU package."""

__version__ = "0.0.2"

from .bindings import (
    PyHexmoveImuData as HexmoveImuData,
    PyHexmoveImuReader as HexmoveImuReader,
)
