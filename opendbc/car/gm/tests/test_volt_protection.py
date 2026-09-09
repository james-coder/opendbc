from dataclasses import replace
import pytest

from opendbc.car import structs
from opendbc.car.gm.volt_protection import ProtectionAuthority, ProtectionGate


AUTHORITY = ProtectionAuthority('a'*64, (0, 100, 200, 300, 400), (0., 1., 2., 3., 4.), 200, .15)


@pytest.mark.parametrize('fault', ['inactive', 'brakePressed', 'gasPressed', 'regenBraking', 'canValid', 'missing_authority'])
def test_protection_cannot_bypass_driver_or_authority(fault):
  cs = structs.CarState.new_message(canValid=True, vEgoRaw=10.)
  cc = structs.CarControl.new_message()
  request = cc.longitudinalProtection
  request.active, request.state = True, 'emergency'
  request.accelCeiling, request.brakeFloor = -4., 400
  request.profileId, request.monoTime, request.observationMonoTime = AUTHORITY.profile_id, 1_000_000_000, 1_000_000_000
  request.assessed = True
  gate = ProtectionGate(None if fault == 'missing_authority' else AUTHORITY)
  if fault in ('brakePressed', 'gasPressed', 'regenBraking', 'canValid'):
    setattr(cs, fault, fault != 'canValid')
  assert not gate.update(request, 1_040_000_000, fault != 'inactive', cs)


def test_unqualified_or_excessive_physical_authority_is_rejected():
  for changes in ({'brake_commands': (0, 400, 401)}, {'brake_decelerations': (0., 1., 2., 3., 5.)},
                  {'observation_age': 1.}, {'profile_id': ''}, {'hold_brake': 0}):
    with pytest.raises(ValueError):
      ProtectionGate(replace(AUTHORITY, **changes))
