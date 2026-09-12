#!/usr/bin/env python3
import unittest

from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


class Buttons:
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  CANCEL = 6


class GmLongitudinalBase(common.CarSafetyTest, common.LongitudinalGasBrakeSafetyTest):

  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB), 2: (0x184,)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd, PSCMStatus

  MAX_POSSIBLE_BRAKE = 2 ** 12
  MAX_BRAKE = 400

  MAX_POSSIBLE_GAS = 4000  # reasonably excessive limits, not signal max
  MIN_POSSIBLE_GAS = -4000

  PCM_CRUISE = False  # openpilot can control the PCM state if longitudinal

  def _send_brake_msg(self, brake):
    values = {"FrictionBrakeCmd": -brake}
    return self.packer_chassis.make_can_msg_safety("EBCMFrictionBrakeCmd", self.BRAKE_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"GasRegenCmd": gas}
    return self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)

  # override these tests from CarSafetyTest, GM longitudinal uses button enable
  def _pcm_status_msg(self, enable):
    raise NotImplementedError

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling and rising edges, respectively.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        with self.subTest(btn_prev=btn_prev, btn_cur=btn_cur):
          self._rx(self._button_msg(btn_prev))
          self.safety.set_controls_allowed(0)
          for _ in range(10):
            self._rx(self._button_msg(btn_cur))

          should_enable = btn_cur != Buttons.DECEL_SET and btn_prev == Buttons.DECEL_SET
          should_enable = should_enable or (btn_cur == Buttons.RES_ACCEL and btn_prev != Buttons.RES_ACCEL)
          should_enable = should_enable and btn_cur != Buttons.CANCEL
          self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())


class TestGmSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  STANDSTILL_THRESHOLD = 10 * 0.0311
  # Ensures ASCM is off on ASCM cars, and relay is not malfunctioning for camera-ACC cars
  RELAY_MALFUNCTION_ADDRS = {0: (0x180,), 2: (0x184,)}  # ASCMLKASteeringCmd, PSCMStatus
  BUTTONS_BUS = 0  # rx or tx
  BRAKE_BUS = 0  # tx only

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 15
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 128
  DRIVER_TORQUE_ALLOWANCE = 65
  DRIVER_TORQUE_FACTOR = 4

  PCM_CRUISE = True  # openpilot is tied to the PCM state if not longitudinal

  EXTRA_SAFETY_PARAM = 0

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, 0)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    if self.PCM_CRUISE:
      values = {"CruiseState": enable}
      return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)
    else:
      raise NotImplementedError

  def _speed_msg(self, speed):
    values = {"%sWheelSpd" % s: speed for s in ["RL", "RR"]}
    return self.packer.make_can_msg_safety("EBCMWheelSpdRear", 0, values)

  def _user_brake_msg(self, brake):
    # GM safety has a brake threshold of 8
    values = {"BrakePedalPos": 8 if brake else 0}
    return self.packer.make_can_msg_safety("ECMAcceleratorPos", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal2": 1 if gas else 0}
    if self.PCM_CRUISE:
      # Fill CruiseState with expected value if the safety mode reads cruise state from gas msg
      values["CruiseState"] = self.safety.get_controls_allowed()
    return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)

  def _torque_driver_msg(self, torque):
    # Safety tests assume driver torque is an int, use DBC factor
    values = {"LKADriverAppldTrq": torque * 0.01}
    return self.packer.make_can_msg_safety("PSCMStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKASteeringCmd": torque, "LKASteeringCmdActive": steer_req}
    return self.packer.make_can_msg_safety("ASCMLKASteeringCmd", 0, values)

  def _button_msg(self, buttons):
    values = {"ACCButtons": buttons}
    return self.packer.make_can_msg_safety("ASCMSteeringButton", self.BUTTONS_BUS, values)


class TestGmEVSafetyBase(TestGmSafetyBase):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV

  # existence of _user_regen_msg adds regen tests
  def _user_regen_msg(self, regen):
    values = {"RegenPaddle": 2 if regen else 0}
    return self.packer.make_can_msg_safety("EBCMRegenPaddle", 0, values)


class TestGmAscmSafety(GmLongitudinalBase, TestGmSafetyBase):
  TX_MSGS = [[0x180, 0], [0x409, 0], [0x40A, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0xA1, 1], [0x306, 1], [0x308, 1], [0x310, 1],  # obs bus
             [0x315, 2]]  # ch bus
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd
  FWD_BUS_LOOKUP: dict[int, int] = {}
  BRAKE_BUS = 2

  MAX_GAS = 1018
  MIN_GAS = -650  # maximum regen
  INACTIVE_GAS = -650

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()


class TestGmAscmEVSafety(TestGmAscmSafety, TestGmEVSafetyBase):
  pass


class TestGmReadOnlyObdSafety(TestGmAscmEVSafety):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV | GMSafetyFlags.READ_ONLY_OBD
  TX_MSGS = TestGmAscmEVSafety.TX_MSGS + [[addr, 0] for addr in range(0x7DF, 0x7E8)]
  REQUESTS = [bytes.fromhex(s) for s in ('02 01 01 00 00 00 00 00', '01 03 00 00 00 00 00 00',
                                       '01 07 00 00 00 00 00 00', '01 0A 00 00 00 00 00 00')]
  FLOW_CONTROL = bytes.fromhex('30 00 0A 00 00 00 00 00')

  def _obd(self, addr=0x7DF, data=None, bus=0):
    return libsafety_py.make_CANPacket(addr, bus, self.REQUESTS[0] if data is None else data)

  def _stationary(self, now=0):
    self.safety.set_timer(now)
    self._rx(self._speed_msg(0))
    self.safety.set_controls_allowed(False)

  def test_obd_all_allowed_frames(self):
    for data in self.REQUESTS:
      self.setUp()
      self._stationary()
      self.assertTrue(self._tx(self._obd(data=data)))
    for addr in range(0x7E0, 0x7E8):
      self.assertTrue(self._tx(self._obd(addr, self.FLOW_CONTROL)))

  def test_obd_rejects_every_other_payload(self):
    # Reset state for each attempt so rate limiting cannot hide a payload-check bug.
    for addr, canonical in [(0x7DF, data) for data in self.REQUESTS] + [(0x7E0, self.FLOW_CONTROL)]:
      for index in range(8):
        for value in range(256):
          payload = bytearray(canonical)
          payload[index] = value
          self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
          self._stationary()
          expected = bytes(payload) in (self.REQUESTS if addr == 0x7DF else [self.FLOW_CONTROL])
          self.assertEqual(expected, self._tx(self._obd(addr, bytes(payload))), (addr, bytes(payload)))

  def test_obd_requires_fresh_valid_speed_and_disengagement(self):
    self.assertFalse(self._tx(self._obd()))  # No wheel-speed observation after initialization.
    self._stationary()
    self.safety.set_timer(500000)
    self.assertFalse(self._tx(self._obd()))
    self._stationary(500000)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._obd()))
    self.safety.set_controls_allowed(False)
    self._rx(self._speed_msg(2))
    self.assertFalse(self._tx(self._obd()))
    self._stationary(500001)
    self.assertTrue(self._tx(self._obd()))
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
    self.assertFalse(self._tx(self._obd()))  # A mode reset must forget the old observation.
    self._rx(libsafety_py.make_CANPacket(0x34A, 1, bytes(5)))
    self.assertFalse(self._tx(self._obd()))
    self._rx(libsafety_py.make_CANPacket(0x34A, 0, bytes(4)))
    self.assertFalse(self._tx(self._obd()))

  def test_obd_requests_and_flow_control_are_rate_limited(self):
    self._stationary()
    self.assertTrue(self._tx(self._obd()))
    self.assertFalse(self._tx(self._obd(data=self.REQUESTS[1])))
    self._stationary(499999)
    self.assertFalse(self._tx(self._obd()))
    self._stationary(500000)
    self.assertTrue(self._tx(self._obd()))
    for addr in range(0x7E0, 0x7E8):
      self.assertTrue(self._tx(self._obd(addr, self.FLOW_CONTROL)))
      self.assertFalse(self._tx(self._obd(addr, self.FLOW_CONTROL)))
    self._stationary(599999)
    self.assertFalse(self._tx(self._obd(0x7E0, self.FLOW_CONTROL)))
    self._stationary(600000)
    self.assertTrue(self._tx(self._obd(0x7E0, self.FLOW_CONTROL)))

  def test_obd_rate_limit_handles_timer_wrap(self):
    self._stationary(0xFFFFFF00)
    self.assertTrue(self._tx(self._obd()))
    self._stationary(100)
    self.assertFalse(self._tx(self._obd()))
    self._stationary(500000)
    self.assertTrue(self._tx(self._obd()))

  def test_obd_bus_length_address_and_flag_scope(self):
    for flags in (0, GMSafetyFlags.EV, GMSafetyFlags.READ_ONLY_OBD,
                  self.EXTRA_SAFETY_PARAM | GMSafetyFlags.HW_CAM):
      self.safety.set_safety_hooks(CarParams.SafetyModel.gm, flags)
      self._stationary()
      self.assertFalse(self._tx(self._obd()))
      self.assertFalse(self._tx(self._obd(0x7E0, self.FLOW_CONTROL)))
    self.setUp()
    self._stationary()
    for bus in (1, 2, 3):
      self.assertFalse(self._tx(self._obd(bus=bus)))
    for size in (0, 1, 2, 3, 4, 5, 6, 7, 12):
      self.assertFalse(self._tx(self._obd(data=self.REQUESTS[0][:size].ljust(size, b'\x00'))))
    for addr in (0x7DE, 0x7E8, 0x18DB33F1):
      self.assertFalse(self._tx(self._obd(addr)))
    for addr in range(0x7E0, 0x7E8):
      for request in self.REQUESTS:
        self.assertFalse(self._tx(self._obd(addr, request)))
    self.assertFalse(self._tx(self._obd(0x7DF, self.FLOW_CONTROL)))

  def test_obd_still_respects_relay_malfunction(self):
    self._stationary()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self._obd()))
    self.assertFalse(self._tx(self._obd(0x7E0, self.FLOW_CONTROL)))


class TestGmReadOnlyDiagnosticsSafety(TestGmReadOnlyObdSafety):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV | GMSafetyFlags.READ_ONLY_OBD | GMSafetyFlags.READ_ONLY_GM_DIAGNOSTICS
  TX_MSGS = TestGmReadOnlyObdSafety.TX_MSGS + [[0x101, 0]]
  GM_REQUEST = bytes.fromhex('FE 03 A9 81 12 00 00 00')
  CONTEXT = [bytes([3, 2, pid, 0, 0, 0, 0, 0]) for pid in (2, 4, 5, 6, 7, 11, 12, 13, 15, 16, 44, 45)] + \
            [bytes([2, 1, pid, 0, 0, 0, 0, 0]) for pid in (5, 12, 44, 45)]

  def test_gm_diagnostic_exact_payload_allowlist(self):
    for addr, payload in [(0x101, self.GM_REQUEST)] + [(0x7E0, data) for data in self.CONTEXT]:
      for index in range(8):
        for value in range(256):
          changed = bytearray(payload)
          changed[index] = value
          self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
          self._stationary()
          expected = bytes(changed) in ([self.GM_REQUEST] if addr == 0x101 else self.CONTEXT)
          self.assertEqual(expected, self._tx(self._obd(addr, bytes(changed))), (addr, bytes(changed)))

  def test_gm_diagnostics_require_new_flag_and_existing_gates(self):
    for addr, data in [(0x101, self.GM_REQUEST), (0x7E0, self.CONTEXT[0])]:
      for flags in (0, 4, 8, 12, 16, 20, 24, 29):
        self.safety.set_safety_hooks(CarParams.SafetyModel.gm, flags)
        self._stationary()
        self.assertFalse(self._tx(self._obd(addr, data)))
      self.setUp()
      self.assertFalse(self._tx(self._obd(addr, data)))
      self._stationary()
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._obd(addr, data)))
      self.safety.set_controls_allowed(False)
      self._rx(self._speed_msg(2))
      self.assertFalse(self._tx(self._obd(addr, data)))
      self._stationary()
      self.safety.set_timer(500000)
      self.assertFalse(self._tx(self._obd(addr, data)))
      self._stationary(500000)
      self.safety.set_relay_malfunction(True)
      self.assertFalse(self._tx(self._obd(addr, data)))

  def test_gm_diagnostics_no_other_bus_address_or_frame_size(self):
    self._stationary()
    for addr, data in [(0x101, self.GM_REQUEST), (0x7E0, self.CONTEXT[0])]:
      for bus in (1, 2, 3):
        self.assertFalse(self._tx(self._obd(addr, data, bus)))
      for size in (0, 1, 2, 3, 4, 5, 6, 7, 12):
        self.assertFalse(self._tx(self._obd(addr, data[:size].ljust(size, b'\x00'))))
    for addr in (0x100, 0x102, 0x241, 0x7DF, 0x7E1, 0x7E7):
      self.assertFalse(self._tx(self._obd(addr, self.GM_REQUEST)))
      self.assertFalse(self._tx(self._obd(addr, self.CONTEXT[0])))

  def test_gm_broadcast_cooldown_and_shared_context_timer(self):
    self._stationary()
    self.assertTrue(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self._stationary(4999999)
    self.assertFalse(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self._stationary(5000000)
    self.assertTrue(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self.assertTrue(self._tx(self._obd(0x7E0, self.CONTEXT[0])))
    self.assertFalse(self._tx(self._obd()))
    self._stationary(5499999)
    self.assertFalse(self._tx(self._obd(0x7E0, self.CONTEXT[1])))
    self._stationary(5500000)
    self.assertTrue(self._tx(self._obd()))
    self.assertFalse(self._tx(self._obd(0x7E0, self.CONTEXT[1])))

  def test_gm_broadcast_timer_wrap_and_reset(self):
    self._stationary(0xFFFFFF00)
    self.assertTrue(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self._stationary(100)
    self.assertFalse(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self._stationary(5000000)
    self.assertTrue(self._tx(self._obd(0x101, self.GM_REQUEST)))
    self.setUp()
    self.assertFalse(self._tx(self._obd(0x101, self.GM_REQUEST)))


class TestGmCameraSafetyBase(TestGmSafetyBase):
  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("ECMEngineStatus", 0, values)


class TestGmCameraSafety(TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180], 0: [0x184]}  # block LKAS message and PSCMStatus
  BUTTONS_BUS = 2  # tx only

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()

  def test_buttons(self):
    # Only CANCEL button is allowed while cruise is enabled
    self.safety.set_controls_allowed(0)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    self.safety.set_controls_allowed(1)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL)))


class TestGmCameraEVSafety(TestGmCameraSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraLongitudinalSafety(GmLongitudinalBase, TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0], [0x315, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180, 0x2CB, 0x370, 0x315], 0: [0x184]}  # block LKAS, ACC messages and PSCMStatus
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB, 0x370, 0x315), 2: (0x184,)}
  BUTTONS_BUS = 0  # rx only

  MAX_GAS = 1346
  MIN_GAS = -540  # maximum regen
  INACTIVE_GAS = -500

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()


class TestGmCameraLongitudinalEVSafety(TestGmCameraLongitudinalSafety, TestGmEVSafetyBase):
  pass


class TestGmIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")

  def _msg(self, mode):
    return self.packer.make_can_msg_safety("BCMGeneralPlatformStatus", 0, {"SystemPowerMode": mode})

  # SystemPowerMode 2=Run, 3=Crank Request
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(2))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
