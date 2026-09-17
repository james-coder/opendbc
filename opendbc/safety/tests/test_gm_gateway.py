"""Parked Object gateway transport; ordinary GM actuation tests are inherited."""
from opendbc.car.structs import CarParams
from opendbc.safety.tests.test_gm import TestGmAscmEVSafety, TestGmReadOnlyObdSafety


class TestGmGatewaySafety(TestGmAscmEVSafety):
  EXTRA_SAFETY_PARAM = 4 | 64
  TX_MSGS = TestGmAscmEVSafety.TX_MSGS + [[0x6F0, 1]]
  _stationary = TestGmReadOnlyObdSafety._stationary
  _obd = TestGmReadOnlyObdSafety._obd

  def request(self, data=b'\x02hi'+bytes(5), addr=0x6F0, bus=1):
    return self._obd(addr, data, bus)

  def test_gateway_scope(self):
    for param in (0, 4, 64, 65, 69, 68):
      for bus in range(4):
        for addr in (0x6EF, 0x6F0, 0x6F1, 0x6F2):
          self.safety.set_safety_hooks(CarParams.SafetyModel.gm, param)
          self._stationary()
          self.assertEqual(param == 68 and bus == 1 and addr == 0x6F0,
                           self._tx(self.request(addr=addr, bus=bus)))

  def test_gateway_freshness_motion_engagement(self):
    self.assertFalse(self._tx(self.request()))
    self._stationary()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self.request()))
    self.safety.set_controls_allowed(False)
    self._rx(self._speed_msg(2))
    self.assertFalse(self._tx(self.request()))
    self._stationary()
    self.safety.set_timer(500000)
    self.assertFalse(self._tx(self.request()))
    self._stationary(500000)
    self.assertTrue(self._tx(self.request()))

  def test_gateway_rate_and_wrap(self):
    self._stationary(0xFFFFFF00)
    self.assertTrue(self._tx(self.request()))
    self._stationary(100)
    self.assertFalse(self._tx(self.request()))
    self._stationary(10000)
    self.assertTrue(self._tx(self.request()))
    self.assertFalse(self._tx(self.request()))

  def test_gateway_framing_and_dlc(self):
    for size in range(9):
      self._stationary(size*20000)
      self.assertEqual(size == 8, self._tx(self.request(data=bytes([2])+bytes(size-1) if size else b'')))
    for first in range(256):
      self._stationary(200000+first*20000)
      data = bytes([first, 10])+bytes(6)
      expected = 1 <= first <= 7 or first in (0x10, 0x11) or 0x20 <= first <= 0x2F
      self.assertEqual(expected, self._tx(self.request(data=data)), hex(first))
    self._stationary(6000000)
    self.assertTrue(self._tx(self.request(data=b'\x30\0\x0a'+bytes(5))))
    self._stationary(6020000)
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self.request()))


del TestGmAscmEVSafety, TestGmReadOnlyObdSafety
