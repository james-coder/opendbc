"""Final Volt protection command checks. No startup I/O or calibration inference."""

from dataclasses import dataclass
import math
import re


@dataclass(frozen=True)
class ProtectionAuthority:
  profile_id: str
  brake_commands: tuple
  brake_decelerations: tuple
  hold_brake: int
  observation_age: float

  def valid(self):
    return (bool(re.fullmatch('[0-9a-f]{64}', self.profile_id))
            and len(self.brake_commands) == len(self.brake_decelerations) >= 2
            and self.brake_commands[0] == self.brake_decelerations[0] == 0
            and self.brake_commands[-1] == 400 and .05 <= self.brake_decelerations[-1] <= 4
            and all(type(b) is int for b in self.brake_commands)
            and all(math.isfinite(d) for d in self.brake_decelerations)
            and all(a < b for a, b in zip(self.brake_commands[:-1], self.brake_commands[1:], strict=True))
            and all(a < b for a, b in zip(self.brake_decelerations[:-1], self.brake_decelerations[1:], strict=True))
            and type(self.hold_brake) is int and 133 <= self.hold_brake <= 400 and 0 < self.observation_age <= .3)

  def floor(self, deceleration):
    # Each entry is a measured LOWER bound across the supported conditions.
    # Round to the next qualified command; do not interpolate an unmeasured one.
    for command, achieved in zip(self.brake_commands, self.brake_decelerations, strict=True):
      if achieved + 1e-6 >= deceleration:
        return command
    return self.brake_commands[-1]


def validate_request(request, authority, now_nanos, active, cs):
  """Return acceptance/reason; caller retains normal panda/driver interlocks."""
  if not request.active:
    return False, 'monitoring'
  if authority is None:
    return False, 'no_qualified_authority'
  if not active or not cs.canValid or cs.gasPressed or cs.brakePressed or cs.regenBraking:
    return False, 'driver_or_can_override'
  try:
    state = str(request.state)
    if request.profileId != authority.profile_id:
      return False, 'calibration_mismatch'
    if not 0 <= now_nanos - request.monoTime <= 150_000_000 or request.monoTime == 0:
      return False, 'stale_command'
    if state not in ('protective', 'emergency', 'degraded', 'holding'):
      return False, 'invalid_state'
    if not math.isfinite(request.accelCeiling) or not -4 <= request.accelCeiling <= 0 or not 0 <= request.brakeFloor <= 400:
      return False, 'out_of_bounds'
    if state in ('protective', 'emergency'):
      if not request.assessed or request.observationMonoTime == 0 or not 0 <= now_nanos-request.observationMonoTime <= authority.observation_age*1e9:
        return False, 'stale_observation'
      expected = 400 if state == 'emergency' else authority.floor(-request.accelCeiling)
      if request.brakeFloor != expected:
        return False, 'unqualified_brake_floor'
    elif state == 'holding' and request.brakeFloor != authority.hold_brake:
      return False, 'unqualified_holding'
    return True, 'accepted'
  except (ValueError, TypeError, AttributeError):
    return False, 'malformed_command'


class ProtectionGate:
  """Independently enforce backend authorization and continuity for degraded hold."""
  def __init__(self, authority=None):
    if authority is not None and not authority.valid():
      raise ValueError('Invalid protection authority')
    self.authority = authority
    self.last = None
    self.last_time = 0
    self.stopped_since = None
    self.accepted, self.reason = False, 'unavailable'
    self.checked_time = self.command_time = 0

  def update(self, request, now_nanos, active, cs):
    self.checked_time, self.command_time = now_nanos, request.monoTime
    stationary = cs.canValid and cs.standstill and math.isfinite(cs.vEgoRaw) and abs(cs.vEgoRaw) < .03
    self.stopped_since = (self.stopped_since if self.stopped_since is not None else now_nanos) if stationary else None
    self.accepted, self.reason = validate_request(request, self.authority, now_nanos, active, cs)
    if self.accepted and str(request.state) == 'degraded':
      if (self.last is None or not 0 <= now_nanos-self.last_time <= 150_000_000
          or request.brakeFloor != self.last[0] or abs(request.accelCeiling-self.last[1]) > 1e-5):
        self.accepted, self.reason = False, 'no_matching_prior_intervention'
    if self.accepted and str(request.state) == 'holding':
      if (self.last is None or not 0 <= now_nanos-self.last_time <= 150_000_000
          or self.stopped_since is None or now_nanos-self.stopped_since < 200_000_000):
        self.accepted, self.reason = False, 'holding_not_confirmed'
    if self.accepted:
      self.last = (request.brakeFloor, request.accelCeiling)
      self.last_time = now_nanos
    if not request.active or not active or not cs.canValid or cs.brakePressed or cs.gasPressed or cs.regenBraking:
      self.last = None
    return self.accepted
