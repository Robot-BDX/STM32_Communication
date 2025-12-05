"""
BDX Robot API Package

High-level Python API for communication with the DM-MC02 board.
"""

from .bdx_robot import (
    BDXRobot,
    IMUData,
    MotorFeedback,
    SystemStatus,
    Telemetry,
    Vector3,
    MotorMode,
    MotorState,
    SystemState,
    list_ports
)

__version__ = '1.0.0'
__all__ = [
    'BDXRobot',
    'IMUData',
    'MotorFeedback',
    'SystemStatus',
    'Telemetry',
    'Vector3',
    'MotorMode',
    'MotorState',
    'SystemState',
    'list_ports'
]
