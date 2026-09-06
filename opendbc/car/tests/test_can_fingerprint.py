import unittest
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import FRAME_FINGERPRINT, can_fingerprint
from opendbc.car.fingerprints import _FINGERPRINTS as FINGERPRINTS
from opendbc.testing import parameterized
from opendbc.car.fingerprints import eliminate_incompatible_cars
from opendbc.car.gm.values import CAR as GM


class TestCanFingerprint(unittest.TestCase):
  def test_gm_identification_request_does_not_eliminate_volt(self):
    expected = GM.CHEVROLET_VOLT
    request = CanData(0x7e3, b'\x02\x1a\xb0\x00\x00\x00\x00\x00', 0)
    can = [CanData(a, b'\x00' * n, 0) for a, n in FINGERPRINTS[expected][0].items()] + [request]
    found, raw = can_fingerprint(lambda **kwargs: [can])
    assert found == expected
    assert raw[0][0x7e3] == 8

  def test_gm_diagnostic_exception_is_narrow(self):
    payload = b'\x02\x1a\xb0\x00\x00\x00\x00\x00'
    for address, data, bus in [(0x7e5, payload, 0), (0x7e3, payload, 1),
                               (0x7e3, payload[:3], 0), (0x7e3, b'\x03' + payload[1:], 0),
                               (0x7e3, b'\x02\x22' + payload[2:], 0)]:
      assert not eliminate_incompatible_cars(CanData(address, data, bus), [GM.CHEVROLET_VOLT])
    other = next(c for c, fps in FINGERPRINTS.items() if c not in GM and all(0x7e3 not in fp for fp in fps))
    assert not eliminate_incompatible_cars(CanData(0x7e3, payload, 0), [other])

  @parameterized("car_model, fingerprints", FINGERPRINTS.items())
  def test_can_fingerprint(self, car_model, fingerprints):
    """Tests online fingerprinting function on offline fingerprints"""

    for fingerprint in fingerprints:  # can have multiple fingerprints for each platform
      can = [CanData(address=address, dat=b'\x00' * length, src=src)
             for address, length in fingerprint.items() for src in (0, 1)]

      fingerprint_iter = iter([can])
      car_fingerprint, finger = can_fingerprint(lambda **kwargs: [next(fingerprint_iter, [])])  # noqa: B023

      assert car_fingerprint == car_model
      assert finger[0] == fingerprint
      assert finger[1] == fingerprint
      assert finger[2] == {}

  def test_timing(self):
    # just pick any CAN fingerprinting car
    car_model = "CHEVROLET_BOLT_EUV"
    fingerprint = FINGERPRINTS[car_model][0]

    cases = []

    # case 1 - one match, make sure we keep going for 100 frames
    can = [CanData(address=address, dat=b'\x00' * length, src=src)
           for address, length in fingerprint.items() for src in (0, 1)]
    cases.append((FRAME_FINGERPRINT, car_model, can))

    # case 2 - no matches, make sure we keep going for 100 frames
    can = [CanData(address=1, dat=b'\x00' * 1, src=src) for src in (0, 1)]  # uncommon address
    cases.append((FRAME_FINGERPRINT, None, can))

    # case 3 - multiple matches, make sure we keep going for 200 frames to try to eliminate some
    can = [CanData(address=2016, dat=b'\x00' * 8, src=src) for src in (0, 1)]  # common address
    cases.append((FRAME_FINGERPRINT * 2, None, can))

    for expected_frames, car_model, can in cases:
      with self.subTest(expected_frames=expected_frames, car_model=car_model):
        frames = 0

        def can_recv(**kwargs):
          nonlocal frames
          frames += 1
          return [can]  # noqa: B023

        car_fingerprint, _ = can_fingerprint(can_recv)
        assert car_fingerprint == car_model
        assert frames == expected_frames + 2  # TODO: fix extra frames
