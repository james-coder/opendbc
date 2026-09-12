from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.test_gm import TestGmAscmEVSafety, TestGmReadOnlyObdSafety, TestGmReadOnlyDiagnosticsSafety


class TestGmEgrSafety(TestGmAscmEVSafety):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV | GMSafetyFlags.READ_ONLY_OBD | GMSafetyFlags.READ_ONLY_GM_DIAGNOSTICS | GMSafetyFlags.READ_ONLY_GM_EGR
  TX_MSGS = TestGmReadOnlyDiagnosticsSafety.TX_MSGS
  REQUESTS = TestGmReadOnlyObdSafety.REQUESTS
  FLOW_CONTROL = TestGmReadOnlyObdSafety.FLOW_CONTROL
  _obd = TestGmReadOnlyObdSafety._obd
  _stationary = TestGmReadOnlyObdSafety._stationary
  NEW = [bytes([2, service, pid, 0, 0, 0, 0, 0]) for service, pids in (
    (1, (0, 1, 5, 11, 12, 15, 16, 32, 44, 45, 51, 64, 65, 96, 105, 107)), (6, (0, 32, 49)), (9, (0, 4, 6))) for pid in pids]
  ALLOWED = NEW + TestGmReadOnlyDiagnosticsSafety.CONTEXT + [FLOW_CONTROL]

  def test_exact_payloads_and_every_single_byte_mutation(self):
    for original in self.NEW:
      for index in range(8):
        for value in range(256):
          data = bytearray(original)
          data[index] = value
          self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
          self._stationary()
          self.assertEqual(bytes(data) in self.ALLOWED, self._tx(self._obd(0x7E0, bytes(data))), bytes(data).hex())

  def test_all_service_pid_pairs(self):
    for service in range(256):
      for pid in range(256):
        data = bytes([2, service, pid, 0, 0, 0, 0, 0])
        self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
        self._stationary()
        self.assertEqual(data in self.ALLOWED, self._tx(self._obd(0x7E0, data)))

  def test_scope_freshness_and_controls(self):
    data = bytes.fromhex('02 06 31 00 00 00 00 00')
    for flags in (0, 28, 32, 36, 44, 52, 56, 61):
      self.safety.set_safety_hooks(CarParams.SafetyModel.gm, flags)
      self._stationary()
      self.assertFalse(self._tx(self._obd(0x7E0, data)))
    self.setUp()
    self.assertFalse(self._tx(self._obd(0x7E0, data)))
    self._stationary()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._obd(0x7E0, data)))
    self.safety.set_controls_allowed(False)
    self._rx(self._speed_msg(2))
    self.assertFalse(self._tx(self._obd(0x7E0, data)))
    self._stationary()
    self.safety.set_timer(500000)
    self.assertFalse(self._tx(self._obd(0x7E0, data)))
    self._stationary(500000)
    for bus in (1, 2, 3):
      self.assertFalse(self._tx(self._obd(0x7E0, data, bus)))
    for address in (0x7DF, 0x7E1, 0x7E7, 0x7E8, 0x18DA10F1):
      self.assertFalse(self._tx(self._obd(address, data)))
    for size in (0, 1, 2, 3, 4, 5, 6, 7, 12):
      self.assertFalse(self._tx(self._obd(0x7E0, data[:size].ljust(size, b'\x00'))))
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self._obd(0x7E0, data)))

  def test_shared_rate_and_wrap(self):
    self._stationary(0xFFFFFF00)
    self.assertTrue(self._tx(self._obd(0x7E0, self.NEW[-1])))
    self._stationary(100)
    self.assertFalse(self._tx(self._obd(0x7E0, self.NEW[0])))
    self._stationary(500000)
    self.assertTrue(self._tx(self._obd()))
    self.assertFalse(self._tx(self._obd(0x7E0, self.NEW[-1])))


# Imported base suites are exercised in test_gm, not collected a second time here.
del TestGmAscmEVSafety, TestGmReadOnlyObdSafety, TestGmReadOnlyDiagnosticsSafety
