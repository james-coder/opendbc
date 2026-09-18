"""Data-derived steady-state following-distance personalization for the Volt.

Independent of the braking/stopping-approach personalization in volt_longitudinal.py
(PROFILE, personal_curve, VoltFlags.PERSONAL/SMOOTH) — this only adjusts the MPC's
per-personality gap-vs-speed constants (t_follow, comfort_brake, stop_distance), the
same three knobs get_T_FOLLOW()/get_safe_obstacle_distance() already use for every car.
It does not touch braking shape, deceleration, or the stopped-gap approach curve.

Derived by tools/profiling/volt_following_distance.py + volt_following_fit.py from the
driver's own manual (openpilot-disengaged) following behavior, and written directly to
the VoltFollowingProfile Params key once the fit's own bounds/monotonicity/held-out-RMSE
checks pass (see following_profile_valid()). Nothing here reads route data or fits
anything at runtime; this module only defines the profile shape and the two gates
(car support, profile validity) that decide whether the fitted numbers apply.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import math

from opendbc.car.gm.values import CAR

# These bounds are THIS FILE'S OWN validation policy, not a stock openpilot safety clip.
# LongitudinalMpc.__init__ separately clips its constructor-default self.stop_distance/
# self.comfort_brake to (4.5,8.)/(1.5,3.) -- but that clip only applies to the fallback
# defaults used when no following_profile is active. Once a following_profile IS active,
# resolve_gap_params() (gap_params.py) feeds these values straight into the solver's
# params[:,6]/[:,7], bypassing that clip entirely. So these bounds are the real backstop
# for this feature, chosen deliberately, not inherited from an untouchable stock limit.
#
# stop_distance floor of 5.5m = this driver's own observed minimum following gap in real
# route data (creeping near-stationary; never lower even then) -- not stock's 4.5m, which
# had no grounding in this driver's actual behavior.
#
# comfort_brake/t_follow upper/lower bounds were widened from stock's (1.5,3.0)/(0.5,2.5)
# after finding stock's shape -- gap(v)/v ("effective seconds") grows ~linearly with speed
# -- makes ALL THREE stock personality levels already far more conservative than even the
# most cautious state driver-manual guidance:
#   "crazy state minimums" (statutory/handbook following-distance guidance, good conditions):
#     Utah/New York/Nevada:      ~2.0s (Utah's is an actual statutory minimum, not just advice)
#     Arizona/California/AAA/NSC: ~3.0s (mainstream US defensive-driving recommendation)
#     Pennsylvania/Texas:         ~4.0s (conservative end of state highway guidance)
#   naturalistic human highway following (actual behavior, not stated intent):
#     25th/50th/75th pct: 1.21s / 1.39s / 1.60s; shrinks toward ~1.2-1.4s as speed rises
#     (FHWA: people BELIEVE they leave ~2.1s; 93% of actual observed gaps were under 1s)
# Stock aggressive alone computes to 3.7s (20mph) - 8.1s (75mph) -- already 2-4x the most
# conservative *legal* guidance (4s), let alone the ~1.4s human-actual median. See
# MANUAL_INTERIM_PROFILE below and docs/ for the full derivation.
STOP_DISTANCE_BOUNDS = (5.5, 8.0)
COMFORT_BRAKE_BOUNDS = (1.5, 5.5)
T_FOLLOW_BOUNDS = (0.15, 2.5)


@dataclass(frozen=True)
class GapParams:
  t_follow: float
  comfort_brake: float
  stop_distance: float


@dataclass(frozen=True)
class VoltFollowingProfile:
  version: str = "volt-following-v1"
  # No fit has ever been applied by default; following_enabled() requires this to
  # flip true, which only tools/profiling/volt_following_fit.py does, and only after
  # its own held-out-route RMSE and bounds/monotonicity checks pass.
  validated: bool = False
  fit_time: str = ""
  heldout_rmse_m: float = math.inf
  aggressive: GapParams = GapParams(1.25, 2.5, 6.0)
  standard: GapParams = GapParams(1.45, 2.5, 6.0)
  relaxed: GapParams = GapParams(1.75, 2.5, 6.0)


FOLLOWING_PROFILE = VoltFollowingProfile()


# Hand-reasoned interim retune (2026-09-17), NOT a data fit -- deployed with the driver in
# the loop after stock aggressive produced a real-world gap the driver and surrounding
# traffic both found unsafe (~7 car lengths at 40mph, other drivers visibly agitated).
# No real driving data existed yet to auto-fit from (see tools/profiling/volt_following_fit.py
# and its "checks_passed" gate) -- this is deliberately a manual, reviewable starting point,
# not something volt_following_fit.py would ever produce on its own; validated=True here is
# a conscious choice by the driver, not an automated-checks pass. Meant to be superseded by
# a real auto-fit once actual following-distance data exists.
#
# Three tiers, each anchored to a concrete real-world reference (see bounds comment above
# for full citations):
#   aggressive -- naturalistic human highway median (~1.4s effective gap)
#   standard   -- Utah's statutory 2.0s minimum (one of the more permissive "crazy state
#                 minimums" -- still far tighter than stock's ~5.4s standard)
#   relaxed    -- UNCHANGED from stock aggressive (1.25, 2.5, 6.0): today's most permissive
#                 stock behavior becomes the new most-conservative option, so nobody who
#                 wants extra caution loses anything they had before.
# stop_distance=5.5 on aggressive/standard is this driver's own observed real-world minimum
# following gap (see STOP_DISTANCE_BOUNDS comment) -- not an arbitrary lower limit.
#
# IMPORTANT CAVEAT (see docs/ or the design doc for the full derivation): the underlying
# formula's own predictions do not match this driver's real observed behavior -- stock
# aggressive predicts ~18 car lengths at 40mph, the driver measured ~7. These numbers are
# directionally reasoned from that formula, not verified to produce a specific literal gap;
# treat as a starting point requiring real on-road confirmation, not a guaranteed outcome.
MANUAL_INTERIM_PROFILE = VoltFollowingProfile(
  version="volt-following-manual-interim-20260917",
  validated=True,
  fit_time=datetime.now(timezone.utc).isoformat(),
  heldout_rmse_m=math.inf,  # explicitly not a data fit; no held-out error exists
  aggressive=GapParams(t_follow=0.25, comfort_brake=4.8, stop_distance=5.5),
  standard=GapParams(t_follow=0.40, comfort_brake=4.2, stop_distance=5.5),
  relaxed=GapParams(t_follow=1.25, comfort_brake=2.5, stop_distance=6.0),
)


def supported(CP):
  """Car-level support only; no gateway/hardware requirement (pure MPC parameters)."""
  return CP.carFingerprint == CAR.CHEVROLET_VOLT and CP.openpilotLongitudinalControl


def _gap_params_valid(gap: GapParams) -> bool:
  return (
    math.isfinite(gap.t_follow) and math.isfinite(gap.comfort_brake) and math.isfinite(gap.stop_distance)
    and T_FOLLOW_BOUNDS[0] <= gap.t_follow <= T_FOLLOW_BOUNDS[1]
    and COMFORT_BRAKE_BOUNDS[0] <= gap.comfort_brake <= COMFORT_BRAKE_BOUNDS[1]
    and STOP_DISTANCE_BOUNDS[0] <= gap.stop_distance <= STOP_DISTANCE_BOUNDS[1]
  )


def following_profile_valid(profile: VoltFollowingProfile) -> bool:
  """Re-checked at load time too (opendbc/car/volt_following_loader in selfdrive), not
  just when the fit script writes it — defense against a stale/corrupted/hand-edited Params
  value reaching the controller."""
  levels = (profile.aggressive, profile.standard, profile.relaxed)
  if not all(_gap_params_valid(level) for level in levels):
    return False
  # A gap should never shrink going aggressive -> standard -> relaxed. t_follow and
  # stop_distance drive distance UP as they increase; comfort_brake is the opposite
  # (it's a divisor in v**2/(2*comfort_brake) -- a HIGHER comfort_brake means a SHORTER
  # gap), so its ascending-monotone direction is reversed relative to the other two.
  ascending = {'t_follow': True, 'comfort_brake': False, 'stop_distance': True}
  for attr, farther_when_larger in ascending.items():
    values = [getattr(level, attr) for level in levels]
    if not farther_when_larger:
      values = [-v for v in values]
    if not (values[0] <= values[1] <= values[2]):
      return False
  return True


def following_enabled(CP, profile: VoltFollowingProfile = FOLLOWING_PROFILE) -> bool:
  return supported(CP) and profile.validated and following_profile_valid(profile)


def resolve_gap_params(personality, profile: VoltFollowingProfile = FOLLOWING_PROFILE) -> GapParams:
  """Pure lookup, independently unit-testable without instantiating the MPC/solver.

  Uses an equality chain, not a dict keyed by the enum members, on purpose: a
  LongitudinalPersonality constructed directly in Python (log.LongitudinalPersonality.aggressive)
  is a plain int, but the same logical value read off a real capnp message (e.g.
  sm['selfdriveState'].personality, as this is actually called in production) is a
  capnp._DynamicEnum with a different __hash__ -- `==` between the two is True, but a dict/set
  `in` check is False, since that only looks at hashes first. A dict lookup here crashed
  plannerd with NotImplementedError on every single drive once a following_profile was ever
  set, undetected because the unit tests only ever passed directly-constructed enum values,
  never a real capnp-message-round-tripped one. gap_params.py's get_T_FOLLOW()/get_jerk_factor()
  already use this same equality-chain pattern for the identical reason -- this now matches."""
  from cereal import log
  if personality == log.LongitudinalPersonality.aggressive:
    return profile.aggressive
  elif personality == log.LongitudinalPersonality.standard:
    return profile.standard
  elif personality == log.LongitudinalPersonality.relaxed:
    return profile.relaxed
  else:
    raise NotImplementedError("Longitudinal personality not supported")
