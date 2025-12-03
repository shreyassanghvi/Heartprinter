"""
Core modules for Heartprinter system.

This package contains fundamental computation and data structures
used throughout the Heartprinter application.
"""

from .compute import (
    MotorCommand,
    HeartprinterStatus,
    MOTOR_STRUCT_FORMAT,
    MOTOR_STRUCT_SIZE,
    STATUS_STRUCT_FORMAT,
    STATUS_STRUCT_SIZE,
    read_all_points,
    compute_plane_mesh,
    triangle_area,
    validate_position,
    compute_heart_frame_transform,
)

__all__ = [
    'MotorCommand',
    'HeartprinterStatus',
    'MOTOR_STRUCT_FORMAT',
    'MOTOR_STRUCT_SIZE',
    'STATUS_STRUCT_FORMAT',
    'STATUS_STRUCT_SIZE',
    'read_all_points',
    'compute_plane_mesh',
    'triangle_area',
    'validate_position',
    'compute_heart_frame_transform',
]
