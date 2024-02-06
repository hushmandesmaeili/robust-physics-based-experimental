from enum import Enum
from typing import List, Dict, Tuple

class TypeOfPhases(Enum):
    STANCE = 1
    FLIGHT = 2

class PreviewPhase:
    def __init__(self, _type: TypeOfPhases, _feet: List[str] = []):
        self.type = _type
        self.feet = _feet
        self.swing_feet = {foot: True for foot in _feet}
        self.step_ = len(_feet) > 0
        self.feet_shift = {}  # Dictionary to hold foot shift data

    def setTypeOfPhase(self, _type: TypeOfPhases):
        self.type = _type

    def setSwingFoot(self, name: str):
        self.swing_feet[name] = True

    def isSwingFoot(self, name: str) -> bool:
        return self.swing_feet.get(name, False)

    def doStep(self) -> bool:
        return self.step_

    def setFootShift(self, name: str, foot_shift: Tuple[float, float]):
        self.feet_shift[name] = foot_shift

    def getTypeOfPhase(self) -> TypeOfPhases:
        return self.type

    def getFootShift(self, name: str) -> Tuple[float, float]:
        return self.feet_shift.get(name, (0.0, 0.0))


class PreviewParams:
    def __init__(self, _duration: float, p_0, p_T, r_0, r_T, alpha_ddot, _phase_type, _id: int = 0):
        # self.id = _id
        self.duration = _duration
        self.p_0 = p_0
        self.p_T = p_T
        self.r_0 = r_0
        self.r_T = r_T
        self.alpha_ddot = alpha_ddot
        self.phase_type = _phase_type  # This will be an instance of PreviewPhase

    def __init__(self):
        pass

class PreviewControl:
    def __init__(self, _params: List[PreviewParams] = []):
        self.params = _params
        self.footplant = None

    def getTotalDuration(self) -> float:
        return sum(param.duration for param in self.params)
    
    def setFootplant(self, footplant):
        self.footplant = footplant