from enum import Enum

class TimeModeEnum(Enum):
    REPLAY = 'replay'
    CAPTURE = 'capture'

    @classmethod
    def from_str(cls, mode):
        if mode == cls.REPLAY.value:
            return cls.REPLAY
        elif mode == cls.CAPTURE.value:
            return cls.CAPTURE
        else:
            return None
