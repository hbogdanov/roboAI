from typing import List, Tuple
from config import IR_MAX, FRONT_THRESH, LEFT_GROUP, RIGHT_GROUP, FRONT_GROUP, BASE_SPEED, AVOID_GAIN

def clamp(v, lo, hi): return max(lo, min(hi, v))

def steer(ir: List[float], base_speed: float = BASE_SPEED) -> Tuple[float, float, float]:
    """
    Returns (left_cmd, right_cmd, front_level) given raw IR readings (None allowed).
    front_level is the normalized front obstacle level [0..1].
    """
    vals = [x if x is not None else 0.0 for x in ir]
    n = [clamp(v / IR_MAX, 0.0, 1.0) for v in vals]

    front = max(n[i] for i in FRONT_GROUP if i < len(n))
    # Turn away if too close
    if front >= FRONT_THRESH:
        left_sum  = sum(n[i] for i in LEFT_GROUP  if i < len(n))
        right_sum = sum(n[i] for i in RIGHT_GROUP if i < len(n))
        if left_sum >= right_sum:
            l, r =  base_speed, -base_speed   # turn right
        else:
            l, r = -base_speed,  base_speed   # turn left
        return (clamp(l, -6.28, 6.28), clamp(r, -6.28, 6.28), front)

    left_sum  = sum(n[i] for i in LEFT_GROUP  if i < len(n))
    right_sum = sum(n[i] for i in RIGHT_GROUP if i < len(n))
    l = base_speed - AVOID_GAIN * right_sum
    r = base_speed - AVOID_GAIN * left_sum
    return (clamp(l, -6.28, 6.28), clamp(r, -6.28, 6.28), front)
