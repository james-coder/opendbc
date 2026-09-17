from unittest.mock import Mock, patch

import pytest

from opendbc.car import car_helpers as helpers, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.gm.values import CAR
from opendbc.car.tests.test_fingerprint_retry import Stream


def run(stream, finger=None, candidate=CAR.CHEVROLET_VOLT):
  finger = gen_empty_fingerprint() if finger is None else finger
  with patch.object(helpers.time, 'monotonic', stream.clock), patch.object(helpers.carlog, 'warning') as log:
    helpers.wait_for_volt_object_headers(candidate, finger, stream.recv)
  return finger, log


@pytest.mark.parametrize('address', [1056, 1120])
def test_delayed_header_prevents_false_dashcam(address):
  # Recorded route: model matched at 1.26 s; headers appeared at 2.20/2.60 s.
  delay = .94 if address == 1056 else 1.34
  stream = Stream(lambda t: [[CanData(address, bytes(8), 1)]] if t >= delay else [])
  finger, log = run(stream)
  assert finger[1][address] == 8 and delay <= stream.now < delay + .02
  cp = helpers.interfaces[CAR.CHEVROLET_VOLT].get_params(CAR.CHEVROLET_VOLT, finger, [], False, False, docs=False)
  assert not cp.radarUnavailable and not cp.dashcamOnly
  assert log.call_args.args[0]['result'] == 'header_received'


@pytest.mark.parametrize('packets', [[], [[CanData(1120, bytes(8), 0)]],
                                    [[CanData(1120, bytes(8), 129)]], [[CanData(1120, bytes(7), 1)]],
                                    [[CanData(1100, bytes(8), 1)]]])
def test_absent_wrong_bus_echo_malformed_and_unrelated_fail_closed(packets):
  stream = Stream(lambda _: packets)
  finger, log = run(stream)
  assert 5 <= stream.now < 5.02 and not finger[1]
  cp = helpers.interfaces[CAR.CHEVROLET_VOLT].get_params(CAR.CHEVROLET_VOLT, finger, [], False, False, docs=False)
  assert cp.radarUnavailable and cp.dashcamOnly
  assert log.call_args.args[0]['result'] == 'timeout_dashcam'


def test_already_present_no_extra_wait():
  finger = gen_empty_fingerprint()
  finger[1][1120] = 8
  stream = Stream(lambda _: pytest.fail('must not consume more traffic'))
  run(stream, finger)
  assert stream.now == 0


@pytest.mark.parametrize('candidate', [None, 'MOCK', CAR.CHEVROLET_BOLT_EUV])
def test_other_models_unchanged(candidate):
  stream = Stream(lambda _: pytest.fail('must not consume traffic'))
  run(stream, candidate=candidate)


def test_malformed_existing_header_not_accepted_and_other_evidence_preserved():
  finger = gen_empty_fingerprint()
  finger[1] = {1120: 7, 300: 8}
  stream = Stream(lambda _: [])
  run(stream, finger)
  assert finger[1] == {300: 8}


def test_late_callback_not_accepted():
  stream = Stream(lambda _: [])

  def recv(wait_for_one=False):
    stream.now += 6
    return [[CanData(1120, bytes(8), 1)]]
  stream.recv = recv
  finger, _ = run(stream)
  assert not finger[1]


@pytest.mark.parametrize('live', [False, True])
def test_get_car_waits_before_params_only_for_live_call(live):
  finger = gen_empty_fingerprint()
  interface = Mock()
  interface.get_params.return_value.carFingerprint = CAR.CHEVROLET_VOLT
  order = []

  def wait(*args):
    order.append('wait')
    finger[1][1120] = 8

  def params(*args, **kwargs):
    order.append('params')
    assert (1120 in args[1][1]) == live
    return interface.get_params.return_value
  interface.get_params.side_effect = params
  with patch.object(helpers, 'fingerprint', return_value=(CAR.CHEVROLET_VOLT, finger, '', [], 0, True)), \
       patch.object(helpers, 'wait_for_volt_object_headers', side_effect=wait), \
       patch.dict(helpers.interfaces, {CAR.CHEVROLET_VOLT: interface}):
    helpers.get_car(Mock(), Mock(), Mock(), False, False, retry_can_fingerprint=live)
  assert order == (['wait', 'params'] if live else ['params'])
