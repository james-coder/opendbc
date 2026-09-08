"""Opt-in Volt actuator prototype; factory ASCM is bypassed, not controlling braking.

Versioned constants are deliberately separate from stock GM tuning. Calibration
and physical validation status live with the profile, not in panda safety flags.
"""

from dataclasses import dataclass
from enum import IntFlag
import math
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.car.gm.values import CAR


class VoltFlags(IntFlag):
  SMOOTH = 1 << 16
  PERSONAL = 1 << 17


@dataclass(frozen=True)
class VoltProfile:
  version: str = "volt-response-20260907-v1-candidate"
  # Until independent response and controlled-vehicle checks pass, the UI must
  # describe these as experimental. No mode is enabled by default.
  validated: bool = False
  speed: tuple = (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0)
  # Conservative monotone envelope of the first-route fit; regen is zero
  # below 1.5 m/s where the CAN evidence shows regeneration disappearing.
  regen: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.035758, 0.035758, 0.703860)
  creep: tuple = (0.400838, 0.275838, 0.150838, 0.025838, 0.0, 0.0, 0.0, 0.0, 0.0)
  brake_gain: float = 0.008518798
  integral_limit: float = 0.45
  braking_ki: float = 0.35
  stop_speed: tuple = (0.0, 0.5, 1.0)
  stop_decel: tuple = (0.18, 0.45, 0.7)
  # Planner defaults stay stock until a reviewed, independently validated fit.
  stop_distance: float = 6.0
  comfort_brake: float = 2.5
  jerk_scale: float = 1.0
  personal_validated: bool = False


PROFILE = VoltProfile()


def supported(CP):
  return CP.carFingerprint == CAR.CHEVROLET_VOLT and CP.networkLocation == CarParams.NetworkLocation.gateway and CP.openpilotLongitudinalControl


def profile_valid(profile=PROFILE):
  return (
    len(profile.speed) == len(profile.regen) == len(profile.creep)
    and len(profile.speed) >= 2
    and all(
      math.isfinite(x)
      for x in (
        *profile.speed,
        *profile.regen,
        *profile.creep,
        profile.brake_gain,
        profile.integral_limit,
        profile.braking_ki,
        profile.stop_distance,
        profile.comfort_brake,
        profile.jerk_scale,
        *profile.stop_speed,
        *profile.stop_decel,
      )
    )
    and profile.speed[0] == 0.0
    and profile.regen[0] == 0.0
    and all(0.0 <= x <= 0.5 for x in profile.creep)
    and all(a < b for a, b in zip(profile.speed, profile.speed[1:], strict=False))
    and all(0 <= a <= b <= 1.5 for a, b in zip(profile.regen, profile.regen[1:], strict=False))
    and 0.003 <= profile.brake_gain <= 0.015
    and 0 <= profile.integral_limit <= 0.6
    and 0 <= profile.braking_ki <= 1
    and 4.5 <= profile.stop_distance <= 8
    and 1.5 <= profile.comfort_brake <= 3
    and 0.5 <= profile.jerk_scale <= 3
    and len(profile.stop_speed) == len(profile.stop_decel) >= 2
    and all(a < b for a, b in zip(profile.stop_speed, profile.stop_speed[1:], strict=False))
    and all(0.1 <= x <= 1.2 for x in profile.stop_decel)
  )


def enabled(CP):
  return supported(CP) and bool(CP.flags & VoltFlags.SMOOTH) and profile_valid()


def personal_enabled(CP):
  return enabled(CP) and bool(CP.flags & VoltFlags.PERSONAL) and PROFILE.personal_validated


def configure(CP, mode):
  """Called once before CarParams is published; CC retains this same CP object."""
  if not supported(CP):
    return "stock"
  CP.flags &= ~int(VoltFlags.SMOOTH | VoltFlags.PERSONAL)
  if mode in ("smooth", "personal") and PROFILE.validated and profile_valid():
    CP.flags |= int(VoltFlags.SMOOTH)
    if mode == "personal" and PROFILE.personal_validated:
      CP.flags |= int(VoltFlags.PERSONAL)
      return "personal"
    return "smooth"
  return "stock"


def allocate(accel, speed, params, *, stopping=False, standstill=False, engine_running=None, pitch=0.0, profile=PROFILE):
  """Return gas/regen and friction commands within existing panda limits.

  Friction supplies the shortfall as regeneration vanishes. Engine state is
  optional: unobserved/engine-on use reduced regenerative authority. The speed
  map describes vehicle response to openpilot commands, not factory ACC logic.
  """
  accel = float(np.clip(accel, params.ACCEL_MIN, params.ACCEL_MAX))
  speed = max(0.0, speed)
  regen = float(np.interp(speed, profile.speed, profile.regen))
  if engine_running is not False:
    regen *= 0.5
  creep = float(np.interp(speed, profile.speed, profile.creep))
  # Calibrated vehicle pitch supplies gravity feedforward, leaving the bounded
  # integral to correct actuator errors instead of absorbing an entire hill.
  gravity = 9.81 * math.sin(float(np.clip(pitch, -0.1, 0.1))) if math.isfinite(pitch) else 0.0
  demand = accel - creep * float(np.interp(accel, [0.0, 0.2], [1.0, 0.0])) + gravity
  gas = float(np.interp(demand, [-max(regen, 0.001), 0.0, params.ACCEL_MAX], [params.MAX_ACC_REGEN, 0.0, params.MAX_GAS]))
  brake = int(round(max(0.0, -demand - regen) / profile.brake_gain))
  if stopping:
    gas = params.INACTIVE_REGEN
  if standstill and stopping:
    # Preserve the stock holding command; do not double holding pressure when
    # switching to a zero-regen map at rest.
    brake = max(brake, int(round(np.interp(-2.0, params.BRAKE_LOOKUP_BP, params.BRAKE_LOOKUP_V))))
  return float(np.clip(gas, params.MAX_ACC_REGEN, params.MAX_GAS)), int(np.clip(brake, 0, params.MAX_BRAKE))
