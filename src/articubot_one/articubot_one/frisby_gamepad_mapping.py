from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class FrisbyDriveCommand:
    left_wheel_m_s: float
    right_wheel_m_s: float
    selected_pwm: int
    right_selected_pwm: int
    turbo: bool


def _item(values: Sequence[float], index: int) -> float:
    return values[index] if 0 <= index < len(values) else 0.0


def pwm_to_wheel_speed(pwm: int, maximum_pwm: int, maximum_speed_m_s: float) -> float:
    if maximum_pwm <= 0 or maximum_speed_m_s <= 0.0:
        raise ValueError('maximum PWM and wheel speed must be positive')
    bounded = max(0, min(int(pwm), int(maximum_pwm)))
    return maximum_speed_m_s * bounded / maximum_pwm


def map_frisby_controls(
    axes: Sequence[float],
    buttons: Sequence[int],
    *,
    deadzone: float = 0.20,
    base_pwm: int = 63,
    right_base_pwm: int | None = None,
    turbo_pwm: int = 120,
    maximum_pwm: int = 150,
    maximum_speed_m_s: float = 0.20,
) -> FrisbyDriveCommand:
    """Map Frisby FGP-38158 input to fixed, non-negative wheel speeds.

    Any meaningful stick/D-pad/face-button motion uses base_pwm.  Holding
    either L1 or R1 (buttons 4/5) selects turbo_pwm without creating motion by
    itself.  Reverse remains intentionally unavailable while direction output
    wiring is disabled.
    """
    if right_base_pwm is None:
        right_base_pwm = base_pwm
    if not 1 <= base_pwm <= turbo_pwm <= maximum_pwm:
        raise ValueError('expected 1 <= base_pwm <= turbo_pwm <= maximum_pwm')
    if not 1 <= right_base_pwm <= turbo_pwm:
        raise ValueError('expected 1 <= right_base_pwm <= turbo_pwm')

    turbo = bool(_item(buttons, 4) or _item(buttons, 5))
    selected_pwm = turbo_pwm if turbo else base_pwm
    right_selected_pwm = turbo_pwm if turbo else right_base_pwm
    left_wheel_speed = pwm_to_wheel_speed(
        selected_pwm, maximum_pwm, maximum_speed_m_s,
    )
    right_wheel_speed = pwm_to_wheel_speed(
        right_selected_pwm, maximum_pwm, maximum_speed_m_s,
    )

    throttle = abs(float(_item(axes, 1))) >= deadzone
    stick_x = float(_item(axes, 0))
    steering = 1 if stick_x >= deadzone else (-1 if stick_x <= -deadzone else 0)

    # ShanWan/Frisby D-pad is normally axes 4/5.
    dpad_x = float(_item(axes, 4))
    dpad_y = float(_item(axes, 5))
    if abs(dpad_y) >= 0.5:
        throttle = True
        steering = 0
    if dpad_x >= 0.5:
        steering = -1
    elif dpad_x <= -0.5:
        steering = 1

    # Face buttons retain button-only control.  L1/R1 are speed selectors only.
    if _item(buttons, 0) or _item(buttons, 1):
        throttle = True
        steering = 0
    elif _item(buttons, 2):
        steering = 1
    elif _item(buttons, 3):
        steering = -1

    left = left_wheel_speed if throttle else 0.0
    right = right_wheel_speed if throttle else 0.0
    if steering > 0:
        left = 0.0
        right = right_wheel_speed
    elif steering < 0:
        left = left_wheel_speed
        right = 0.0
    return FrisbyDriveCommand(
        left, right, selected_pwm, right_selected_pwm, turbo,
    )
