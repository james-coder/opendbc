"""Opt-in Volt actuator prototype; factory ASCM is bypassed, not controlling braking.

Versioned constants are deliberately separate from stock GM tuning. Calibration
and physical validation status live with the profile, not in panda safety flags.
"""

from dataclasses import dataclass
from collections import deque
from enum import IntFlag
import math
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.car.gm.values import CAR


class VoltFlags(IntFlag):
  SMOOTH = 1 << 16
  PERSONAL = 1 << 17
  TEST = 1 << 18
  BUNDLE = 1 << 19


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
  brake_gain_speed: tuple = ()
  brake_deadband: float = 0.0
  brake_power: float = 1.0
  response_horizon: float = 0.4
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


class VoltHold:
  """Confirm stationary wheels before the personal candidate's hold transition."""
  def __init__(self):
    self.elapsed = 0.

  def update(self, standstill, raw_speed, active, dt):
    self.elapsed = self.elapsed + dt if active and standstill and math.isfinite(raw_speed) and abs(raw_speed) < .03 else 0.
    return self.elapsed >= .2-1e-9


class RegenResponse:
  """Compare measured response with delayed demand before reducing regen credit.

  The delay/filter avoids treating normal pressure buildup as missing regen.
  Correction is continuous in deceleration error, without a late step after a
  fixed settled-command timer. This is a response observer, not battery sensing.
  """
  def __init__(self):
    self.scale = 1.0
    self.deficit_time = 0.0
    self.previous = None
    self.history = deque()
    self.expected = 0.

  def update(self, requested, measured, speed, active, dt):
    if not active:
      self.__init__()
      return self.scale
    if not all(math.isfinite(x) for x in (requested, measured, speed, dt)) or dt <= 0:
      return self.scale
    self.history.append(requested)
    delayed = self.history.popleft() if len(self.history) > max(1, round(.4 / dt)) else 0.
    self.expected += dt / (.2 + dt) * (delayed - self.expected)
    self.previous = requested
    error = measured - self.expected
    if speed >= .5 and requested < -.2 and self.expected < -.2 and error > .15:
      self.deficit_time += dt
      self.scale = max(0., self.scale - min(.75, .6 * (error - .15)) * dt)
    else:
      self.deficit_time = 0.0
    if requested < -.2 and error < -.35:
      self.scale = min(1., self.scale + min(.5, .4 * (-error - .35)) * dt)
    return self.scale


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
        *profile.brake_gain_speed,
        profile.brake_deadband,
        profile.brake_power,
        profile.response_horizon,
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
    and (not profile.brake_gain_speed or len(profile.brake_gain_speed) == len(profile.speed)
         and all(0.001 <= value <= 0.05 for value in profile.brake_gain_speed))
    and 0 <= profile.brake_deadband <= 60
    and 1 <= profile.brake_power <= 3
    and 0 <= profile.response_horizon <= .6
    and 0 <= profile.integral_limit <= 0.6
    and 0 <= profile.braking_ki <= 1
    and 4.5 <= profile.stop_distance <= 8
    and 1.5 <= profile.comfort_brake <= 3
    and 0.5 <= profile.jerk_scale <= 3
    and len(profile.stop_speed) == len(profile.stop_decel) >= 2
    and all(a < b for a, b in zip(profile.stop_speed, profile.stop_speed[1:], strict=False))
    and all(0.1 <= x <= 2.5 for x in profile.stop_decel)
  )


def enabled(CP):
  return supported(CP) and bool(CP.flags & VoltFlags.SMOOTH) and profile_valid()


def personal_enabled(CP):
  return enabled(CP) and bool(CP.flags & VoltFlags.PERSONAL) and PROFILE.personal_validated


def configure(CP, mode, *, profile=PROFILE, test_ready=False, kind='personal'):
  """Called once before CarParams is published; CC retains this same CP object."""
  if not supported(CP):
    return "stock"
  CP.flags &= ~int(VoltFlags.SMOOTH | VoltFlags.PERSONAL | VoltFlags.TEST | VoltFlags.BUNDLE)
  if (kind not in ('brake', 'personal') or mode == 'personal' and kind != 'personal'
      or mode == 'smooth' and kind != 'brake'):
    return 'stock'
  if mode == 'test' and test_ready and profile_valid(profile):
    CP.flags |= int(VoltFlags.SMOOTH | VoltFlags.TEST | VoltFlags.BUNDLE)
    if kind == 'personal':
      CP.flags |= int(VoltFlags.PERSONAL)
    return 'test'
  if mode in ("smooth", "personal") and profile.validated and profile_valid(profile):
    CP.flags |= int(VoltFlags.SMOOTH | VoltFlags.BUNDLE)
    if mode == "personal" and profile.personal_validated:
      CP.flags |= int(VoltFlags.PERSONAL)
      return "personal"
    return "smooth"
  return "stock"


def allocate(accel, speed, params, *, stopping=False, standstill=False, engine_running=None, pitch=0.0, profile=PROFILE, regen_scale=1.0,
             measured_accel=0.0):
  """Return gas/regen and friction commands within existing panda limits.

  Friction supplies the shortfall as regeneration vanishes. Engine state is
  optional: unobserved/engine-on use reduced regenerative authority. The speed
  map describes vehicle response to openpilot commands, not factory ACC logic.
  """
  accel = float(np.clip(accel, params.ACCEL_MIN, params.ACCEL_MAX))
  speed = max(0.0, speed)
  predicted_speed = max(0., speed + min(0., measured_accel) * profile.response_horizon) if math.isfinite(measured_accel) else speed
  regen = float(np.interp(predicted_speed, profile.speed, profile.regen))
  regen *= float(np.clip(regen_scale, 0., 1.))
  if engine_running is not False:
    regen *= 0.5
  creep = float(np.interp(speed, profile.speed, profile.creep))
  # Calibrated vehicle pitch supplies gravity feedforward, leaving the bounded
  # integral to correct actuator errors instead of absorbing an entire hill.
  gravity = 9.81 * math.sin(float(np.clip(pitch, -0.1, 0.1))) if math.isfinite(pitch) else 0.0
  demand = accel - creep * float(np.interp(accel, [0.0, 0.2], [1.0, 0.0])) + gravity
  gas = float(np.interp(demand, [-max(regen, 0.001), 0.0, params.ACCEL_MAX], [params.MAX_ACC_REGEN, 0.0, params.MAX_GAS]))
  friction = max(0.0, -demand - regen)
  # Pressure will build after the vehicle has slowed. Using the present speed
  # here overcommands friction as the low-speed pressure gain rises.
  gain = float(np.interp(predicted_speed, profile.speed, profile.brake_gain_speed)) if profile.brake_gain_speed else profile.brake_gain
  exponent = 1 + (profile.brake_power - 1) * min(1., predicted_speed / 3.)
  brake = int(round(400 * (friction / (400 * gain)) ** (1 / exponent) + (profile.brake_deadband if friction > 0 else 0.)))
  if stopping:
    gas = params.INACTIVE_REGEN
  if standstill and stopping:
    # Preserve the stock holding command; do not double holding pressure when
    # switching to a zero-regen map at rest.
    brake = max(brake, int(round(np.interp(-2.0, params.BRAKE_LOOKUP_BP, params.BRAKE_LOOKUP_V))))
  return float(np.clip(gas, params.MAX_ACC_REGEN, params.MAX_GAS)), int(np.clip(brake, 0, params.MAX_BRAKE))
