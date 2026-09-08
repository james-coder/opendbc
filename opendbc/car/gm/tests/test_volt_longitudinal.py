from dataclasses import replace
import math

from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import CAR, CarControllerParams
from opendbc.car.gm.volt_longitudinal import PROFILE, VoltFlags, RegenResponse, allocate, configure, enabled, profile_valid


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


def test_regen_observer_allows_actuator_lag_then_corrects_continuously():
  response = RegenResponse()
  for _ in range(10):
    assert response.update(-1., 0., 5., True, .04) == 1.
  scales = [response.update(-1., 0., 5., True, .04) for _ in range(100)]
  assert all(0 <= a - b <= .75 * .04 + 1e-8 for a, b in zip(scales, scales[1:], strict=False))
  assert response.scale == 0.
  for _ in range(120):
    response.update(-1., -2., 5., True, .04)
  assert response.scale == 1.
  response.update(0., 0., 0., False, .04)
  assert response.scale == 1. and response.previous is None


def test_regen_observer_does_not_penalize_expected_pressure_response():
  response = RegenResponse()
  for _ in range(100):
    response.update(-1., response.expected, 5., True, .04)
  assert response.scale == 1.


def test_regen_fade_is_anticipated_before_capacity_disappears():
  p = CarControllerParams(CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT))
  profile = replace(PROFILE, regen=tuple(min(1., v / 5) for v in PROFILE.speed))
  now = allocate(-1., 2., p, profile=profile, engine_running=False)
  approaching = allocate(-1., 2., p, profile=profile, measured_accel=-1., engine_running=False)
  assert approaching[1] > now[1]


def test_supervised_test_selection_requires_qualification_and_keeps_road_validation_separate():
  cp = CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT)
  assert configure(cp, 'test') == 'stock'
  assert configure(cp, 'test', test_ready=True) == 'test'
  assert cp.flags & VoltFlags.TEST
  assert configure(cp, 'personal', test_ready=True) == 'stock'
  assert not cp.flags & VoltFlags.TEST


def test_speed_dependent_friction_calibration_is_bounded():
  params = CarControllerParams(CarInterface.get_non_essential_params(CAR.CHEVROLET_VOLT))
  profile = replace(PROFILE, brake_gain_speed=tuple(.01 for _ in PROFILE.speed), brake_deadband=40.)
  assert profile_valid(profile)
  assert not profile_valid(replace(profile, brake_gain_speed=(float('nan'),) * len(PROFILE.speed)))
  assert not profile_valid(replace(profile, brake_deadband=61.))
  for speed in (0., .5, 5., 20.):
    for scale in (0., .5, 1.):
      previous = 400
      for i in range(601):
        gas, brake = allocate(-4. + i * .01, speed, params, profile=profile, regen_scale=scale)
        assert -650 <= gas <= 1018 and 0 <= brake <= previous
        previous = brake
