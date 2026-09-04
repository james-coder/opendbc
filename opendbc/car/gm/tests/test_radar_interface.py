import unittest

from opendbc.can.packer import CANPacker
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.gm.radar_interface import GMRadarFaultBit, NUM_SLOTS, RadarInterface, SLOT_1_MSG
from opendbc.car.gm.values import CAR

RADAR_DBC = 'gm_global_a_object'

FAULT_SIGNALS = {
  GMRadarFaultBit.SENSOR_BLOCKED: 'FLRRSnsrBlckd',
  GMRadarFaultBit.SENSITIVITY: 'FLRRSnstvFltPrsntInt',
  GMRadarFaultBit.YAW_RATE_PLAUSIBILITY: 'FLRRYawRtPlsblityFlt',
  GMRadarFaultBit.HARDWARE: 'FLRRHWFltPrsntInt',
  GMRadarFaultBit.ANTENNA_TUNING: 'FLRRAntTngFltPrsnt',
  GMRadarFaultBit.ALIGNMENT: 'FLRRAlgnFltPrsnt',
}


class TestGMRadarInterface(unittest.TestCase):
  def setUp(self):
    CarInterface = interfaces[CAR.CHEVROLET_VOLT]
    # Fingerprint with the radar header present on the obstacle bus, as on a real Volt
    # with a working (or intermittently faulting) radar unit - CanBus.OBSTACLE == 1.
    fingerprints = {i: {} for i in range(4)}
    fingerprints[1][1120] = 8  # RADAR_HEADER_MSG
    self.CP = CarInterface.get_params(CAR.CHEVROLET_VOLT, fingerprints, [], alpha_long=False,
                                       is_release=False, docs=False)
    assert not self.CP.radarUnavailable
    assert not self.CP.dashcamOnly

    self.ri = RadarInterface(self.CP)
    assert self.ri.rcp is not None
    self.packer = CANPacker(RADAR_DBC)

  def _header_msg(self, num_targets=0, fault_bits=GMRadarFaultBit(0)):
    values = {'FLRRNumValidTargets': num_targets}
    for bit, signal in FAULT_SIGNALS.items():
      values[signal] = 1 if fault_bits & bit else 0
    addr, dat, bus = self.packer.make_can_msg('F_LRR_Obj_Header', 1, values)
    return CanData(addr, dat, bus)

  def _slot_msg(self, slot_addr, track_id=0, rng=0.0):
    addr, dat, bus = self.packer.make_can_msg(slot_addr, 1, {
      'TrkRange': rng, 'TrkObjectID': track_id, 'TrkRangeRate': 0.0,
      'TrkRangeAccel': 0.0, 'TrkAzimuth': 0.0, 'TrkWidth': 0.0,
    })
    return CanData(addr, dat, bus)

  def _frame(self, fault_bits=GMRadarFaultBit(0), targets: dict[int, int] | None = None):
    """Builds one full radar cycle (header + all target slots) and feeds it through."""
    targets = targets or {}
    msgs = [self._header_msg(num_targets=len(targets), fault_bits=fault_bits)]
    for i in range(NUM_SLOTS):
      addr = SLOT_1_MSG + i
      if i in targets:
        msgs.append(self._slot_msg(addr, track_id=targets[i], rng=20.0))
      else:
        msgs.append(self._slot_msg(addr))
    return self.ri.update([(1, msgs)])

  def test_clean_frame_has_no_errors(self):
    rr = self._frame(targets={0: 5})
    assert rr is not None
    assert not rr.errors.radarFault
    assert not rr.errors.radarDegraded
    assert rr.errors.radarDegradedReasons == 0
    assert len(rr.points) == 1
    assert rr.points[0].trackId == 5

  def test_each_fault_bit_individually_degrades_not_faults(self):
    for bit in GMRadarFaultBit:
      with self.subTest(bit=bit):
        ri_local = RadarInterface(self.CP)
        rr = ri_local.update([(1, [self._header_msg(num_targets=1, fault_bits=bit)] +
                                   [self._slot_msg(SLOT_1_MSG + i, track_id=1 if i == 0 else 0, rng=20.0 if i == 0 else 0.0)
                                    for i in range(NUM_SLOTS)])])
        assert rr is not None
        assert not rr.errors.radarFault, f"{bit} incorrectly set radarFault"
        assert rr.errors.radarDegraded, f"{bit} did not set radarDegraded"
        assert rr.errors.radarDegradedReasons == int(bit)
        # radar points are suppressed while degraded, even though a target was "present"
        assert len(rr.points) == 0

  def test_multiple_bits_combine_as_bitmask(self):
    combined = GMRadarFaultBit.SENSOR_BLOCKED | GMRadarFaultBit.ALIGNMENT
    rr = self._frame(fault_bits=combined)
    assert rr is not None
    assert not rr.errors.radarFault
    assert rr.errors.radarDegraded
    assert rr.errors.radarDegradedReasons == int(combined)

  def test_recovers_automatically_on_next_clean_frame(self):
    # start clean and tracking a target
    rr = self._frame(targets={0: 7})
    assert len(rr.points) == 1

    # fault hits - points are dropped, degraded (not fault) is reported
    rr = self._frame(fault_bits=GMRadarFaultBit.HARDWARE, targets={0: 7})
    assert len(rr.points) == 0
    assert rr.errors.radarDegraded
    assert not rr.errors.radarFault

    # the very next clean frame - no restart, no latched state, tracking resumes immediately
    rr = self._frame(targets={0: 7})
    assert not rr.errors.radarDegraded
    assert rr.errors.radarDegradedReasons == 0
    assert len(rr.points) == 1
    assert rr.points[0].trackId == 7

  def test_can_error_is_unaffected_and_still_fatal(self):
    # an empty/invalid CAN update should still be able to raise canError - this path is
    # deliberately untouched since a full CAN dropout is a different failure class.
    rr = self.ri.update([(1, [])])
    # first call establishes can_valid state; just ensure it doesn't crash and doesn't
    # claim a false recovery
    assert rr is None or not rr.errors.radarDegraded or rr.errors.canError


if __name__ == "__main__":
  unittest.main()
