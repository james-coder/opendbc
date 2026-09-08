from dataclasses import replace
import math

from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, CarControllerParams
from opendbc.car.gm.volt_longitudinal import PROFILE, VoltFlags, allocate, configure, enabled, profile_valid


def test_unvalidated_profile_is_locked_at_startup():
  cp = CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT)
  assert configure(cp, "smooth") == "stock"
  assert configure(cp, "personal") == "stock"
  assert not enabled(cp)
  assert not profile_valid(replace(PROFILE, brake_gain=float("nan")))
  assert not profile_valid(replace(PROFILE, speed=(), regen=(), creep=()))


def test_low_speed_brake_blending_and_limits():
  params = CarControllerParams(CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT))
  gas, brake = allocate(-0.5, 0.6, params, stopping=True, engine_running=False)
  assert gas == -650 and brake > 0
  for speed in (0.0, 0.5, 1.0, 2.0, 10.0, 20.0):
    for pitch in (-0.1, 0.0, 0.1, float("nan")):
      previous = (-650.0, 400)
      for i in range(601):
        value = allocate(-4.0 + i * 0.01, speed, params, pitch=pitch)
        assert math.isfinite(value[0]) and -650 <= value[0] <= 1018 and 0 <= value[1] <= 400
        assert value[0] >= previous[0] and value[1] <= previous[1]
        previous = value


def test_empty_can_is_not_engine_off_or_a_crash():
  cp = CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT)
  interface = CarInterface(cp)
  interface.update([])
  assert interface.CS.volt_engine_running is None
  assert interface.CC.CP is cp


def test_flags_are_scoped_to_the_bypassed_volt_installation():
  cp = CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT)
  cp.flags |= int(VoltFlags.SMOOTH)
  assert enabled(cp)
  cp.carFingerprint = CAR.CHEVROLET_BOLT_EUV
  assert not enabled(cp)
