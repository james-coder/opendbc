from dataclasses import replace

import pytest

from opendbc.car import structs
from opendbc.car.gm.values import CAR
from opendbc.car.gm.volt_following import (
  GapParams, VoltFollowingProfile, MANUAL_INTERIM_PROFILE, following_enabled, following_profile_valid,
  resolve_gap_params, supported, STOP_DISTANCE_BOUNDS, COMFORT_BRAKE_BOUNDS, T_FOLLOW_BOUNDS,
)
from cereal import log


def make_cp(fingerprint=CAR.CHEVROLET_VOLT, long_control=True):
  return structs.CarParams.new_message(carFingerprint=fingerprint, openpilotLongitudinalControl=long_control)


FITTED = VoltFollowingProfile(
  validated=True,
  # comfort_brake descends aggressive->relaxed on purpose: it's a divisor in the gap
  # formula, so a HIGHER comfort_brake means a SHORTER (more aggressive) gap.
  aggressive=GapParams(1.0, 2.5, 5.5),
  standard=GapParams(1.2, 2.2, 6.0),
  relaxed=GapParams(1.6, 2.0, 6.5),
)


def test_supported_requires_volt_and_longitudinal_control():
  assert supported(make_cp())
  assert not supported(make_cp(fingerprint=CAR.CHEVROLET_BOLT_EUV))
  assert not supported(make_cp(long_control=False))


def test_default_profile_is_unvalidated_and_stock_shaped():
  default = VoltFollowingProfile()
  assert not default.validated
  assert following_profile_valid(default)  # bounds/monotonicity still hold even though unvalidated
  assert not following_enabled(make_cp(), default)  # validated=False blocks it regardless of car


def test_following_enabled_requires_volt_validated_and_valid_profile():
  assert following_enabled(make_cp(), FITTED)
  assert not following_enabled(make_cp(fingerprint=CAR.CHEVROLET_BOLT_EUV), FITTED)
  assert not following_enabled(make_cp(), replace(FITTED, validated=False))


@pytest.mark.parametrize('attr', ['t_follow', 'comfort_brake', 'stop_distance'])
def test_profile_invalid_if_any_level_breaks_bounds(attr):
  bad = replace(FITTED, aggressive=replace(FITTED.aggressive, **{attr: 100.0}))
  assert not following_profile_valid(bad)


@pytest.mark.parametrize('attr', ['t_follow', 'comfort_brake', 'stop_distance'])
def test_profile_invalid_if_not_monotone_aggressive_to_relaxed(attr):
  # comfort_brake's "farther" direction is reversed (it's a divisor): break monotonicity
  # by pushing aggressive further FROM the direction that would make it a shorter gap.
  # Small delta (not +/-1.0) so this exercises the monotonicity check specifically,
  # not also tripping the separate bounds check.
  delta = -0.2 if attr == 'comfort_brake' else 0.1
  swapped = replace(FITTED, aggressive=replace(FITTED.aggressive, **{attr: getattr(FITTED.relaxed, attr) + delta}))
  # Confirm it's specifically the monotonicity check catching this, not the bounds check.
  bounds = {'t_follow': T_FOLLOW_BOUNDS, 'comfort_brake': COMFORT_BRAKE_BOUNDS, 'stop_distance': STOP_DISTANCE_BOUNDS}[attr]
  assert bounds[0] <= getattr(swapped.aggressive, attr) <= bounds[1]
  assert not following_profile_valid(swapped)


def test_profile_invalid_on_nan_or_inf():
  broken = replace(FITTED, aggressive=replace(FITTED.aggressive, t_follow=float('nan')))
  assert not following_profile_valid(broken)


def test_resolve_gap_params_maps_each_personality():
  assert resolve_gap_params(log.LongitudinalPersonality.aggressive, FITTED) == FITTED.aggressive
  assert resolve_gap_params(log.LongitudinalPersonality.standard, FITTED) == FITTED.standard
  assert resolve_gap_params(log.LongitudinalPersonality.relaxed, FITTED) == FITTED.relaxed


@pytest.mark.parametrize('personality,expected', [
  (log.LongitudinalPersonality.aggressive, 'aggressive'),
  (log.LongitudinalPersonality.standard, 'standard'),
  (log.LongitudinalPersonality.relaxed, 'relaxed'),
])
def test_resolve_gap_params_works_with_a_real_capnp_message_round_trip(personality, expected):
  """Regression test for a real production crash: log.LongitudinalPersonality.aggressive
  constructed directly in Python is a plain int, but the same logical value read off a real
  capnp message (e.g. sm['selfdriveState'].personality, exactly how this is actually called
  from long_mpc.py) is a capnp._DynamicEnum with a different __hash__. `==` between the two
  is True, but a dict/set `in` check is False. A previous dict-based implementation of
  resolve_gap_params() passed test_resolve_gap_params_maps_each_personality() above (which
  only ever used directly-constructed values) while crashing plannerd with
  NotImplementedError on every single real drive with a following_profile active -- this
  test exists specifically because that one didn't catch it."""
  msg = log.SelfdriveState.new_message()
  msg.personality = personality
  with log.SelfdriveState.from_bytes(msg.to_bytes()) as decoded:
    wire_value = decoded.personality
    assert type(wire_value).__name__ == '_DynamicEnum'  # confirms this actually exercises the real bug path
    assert resolve_gap_params(wire_value, FITTED) == getattr(FITTED, expected)


def test_manual_interim_profile_is_valid_and_enabled():
  """The 2026-09-17 hand-reasoned emergency retune (not a data fit) -- must itself pass
  every check any auto-fit would have to pass. Regression coverage for the specific real
  numbers being deployed, on top of the generic property tests above."""
  assert MANUAL_INTERIM_PROFILE.validated
  assert following_profile_valid(MANUAL_INTERIM_PROFILE)
  assert following_enabled(make_cp(), MANUAL_INTERIM_PROFILE)
  assert not following_enabled(make_cp(fingerprint=CAR.CHEVROLET_BOLT_EUV), MANUAL_INTERIM_PROFILE)


def test_manual_interim_profile_relaxed_matches_old_stock_aggressive():
  """Explicit design intent: today's stock aggressive (1.25, 2.5, 6.0) becomes the new
  most-conservative "relaxed" tier, so nobody who wants extra caution loses anything."""
  assert MANUAL_INTERIM_PROFILE.relaxed == GapParams(1.25, 2.5, 6.0)


def test_manual_interim_profile_stop_distance_respects_new_floor():
  """stop_distance=5.5 on aggressive/standard is the driver's own observed real-world
  minimum, not the old stock-inherited 4.5m floor."""
  assert MANUAL_INTERIM_PROFILE.aggressive.stop_distance == STOP_DISTANCE_BOUNDS[0] == 5.5
  assert MANUAL_INTERIM_PROFILE.standard.stop_distance == 5.5


def test_manual_interim_profile_shortens_the_gap_at_speed():
  """The actual point of this whole retune: confirm aggressive/standard produce a
  meaningfully shorter gap than stock at a representative speed (40mph, the driver's
  reported complaint), not just that the numbers are individually valid."""
  from selfdrive.controls.lib.longitudinal_mpc_lib.gap_params import get_safe_obstacle_distance
  v40 = 40 * 0.44704
  stock_aggressive_gap = get_safe_obstacle_distance(v40, 1.25, 6.0, 2.5)
  new_aggressive_gap = get_safe_obstacle_distance(
    v40, MANUAL_INTERIM_PROFILE.aggressive.t_follow, MANUAL_INTERIM_PROFILE.aggressive.stop_distance,
    MANUAL_INTERIM_PROFILE.aggressive.comfort_brake)
  new_standard_gap = get_safe_obstacle_distance(
    v40, MANUAL_INTERIM_PROFILE.standard.t_follow, MANUAL_INTERIM_PROFILE.standard.stop_distance,
    MANUAL_INTERIM_PROFILE.standard.comfort_brake)
  assert new_aggressive_gap < new_standard_gap < stock_aggressive_gap
