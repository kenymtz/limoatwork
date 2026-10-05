from enum import Enum, auto

class MissionState(Enum):
    INIT = auto()
    SET_INITIAL_POSE = auto()
    WAIT_LOCALIZATION = auto()
    LOAD_NEXT_MISSION = auto()
    NAVIGATING = auto()
    DOCKING = auto()
    UNDOCKING = auto()
    FINISHED = auto()
    ERROR = auto()

class DockingState(Enum):
    IDLE = auto()
    SEARCH = auto()
    ALIGNING_COARSE = auto()
    ALIGNING_FINE = auto()
    RECENTER_FOR_DOCK = auto()
    APPROACHING = auto()
    VISION_ACQUIRE_TARGET = auto()
    VISION_ALIGN_COARSE = auto()
    VISION_ALIGN_FINE = auto()
    VISION_READY = auto()
    VISION_APPROACHING = auto()
    DONE = auto()
    FAILED = auto()
