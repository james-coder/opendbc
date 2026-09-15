from unittest.mock import Mock, patch

import pytest

from opendbc.car import car_helpers as helpers
from opendbc.car.can_definitions import CanData
from opendbc.car.fingerprints import _FINGERPRINTS
from opendbc.car.gm.values import CAR


def messages(model=CAR.CHEVROLET_VOLT, bus=0):
  return [CanData(a, bytes(n), bus) for a, n in _FINGERPRINTS[model][0].items()]


class Stream:
  def __init__(self, packets):
    self.now = 0.
    self.packets = packets

  def clock(self):
    return self.now

  def recv(self, wait_for_one=False):
    self.now += .01
    return self.packets(self.now)


def run(stream, **kwargs):
  with patch.object(helpers.time, 'monotonic', stream.clock), patch.object(helpers.carlog, 'warning') as log:
    result = helpers.can_fingerprint_with_retries(stream.recv, **kwargs)
  return result, [c.args[0] for c in log.call_args_list]


@pytest.mark.parametrize('clear_at,attempts', [(0., 1), (2., 2), (5., 3)])
def test_transient_recovery(clear_at, attempts):
  clean = messages()
  bad = clean + [CanData(0x252, bytes(8), 0), CanData(0x552, bytes(8), 0)]
  stream = Stream(lambda t: [bad if t < clear_at else clean])
  counts = []
  (candidate, finger), logs = run(stream, on_attempt=counts.append)
  assert candidate == CAR.CHEVROLET_VOLT
  assert counts == list(range(1, attempts + 1))
  assert len(logs) == attempts
  assert finger[0] == _FINGERPRINTS[CAR.CHEVROLET_VOLT][0]
  assert stream.now < (1.1 if attempts == 1 else 9.1)
  if attempts > 1:
    assert logs[-1]['elapsed'] >= 2.
    rejection = next(e for e in logs[0]['eliminations'] if CAR.CHEVROLET_VOLT in e['candidates'] and e['bus'] == 0)
    assert rejection['address'] == 0x252
    assert rejection['data'] == '0000000000000000'
    assert logs[0]['candidate'] is None


@pytest.mark.parametrize('kind', ['unknown', 'wrong_length', 'ambiguous', 'absent', 'conflicting'])
def test_persistent_failure_is_bounded(kind):
  packets = {
    'unknown': [messages() + [CanData(0x252, bytes(8), 0)]],
    'wrong_length': [messages() + [CanData(messages()[0].address, b'', 0)]],
    'ambiguous': [[CanData(0x7e0, bytes(8), 0)]],
    'absent': [],
    'conflicting': [messages() + messages(CAR.CHEVROLET_BOLT_EUV, 1)],
  }[kind]
  stream = Stream(lambda _: packets)
  (candidate, _), logs = run(stream)
  assert candidate is None
  assert len(logs) == 3
  assert stream.now <= 11.04  # receive timeout can straddle a wall-clock deadline
  assert stream.now - logs[0]['elapsed'] <= 8.03
  if kind == 'conflicting':
    assert all(e['reason'] == 'conflicting_buses' for e in logs)


def test_entire_batch_checked_before_acceptance():
  stream = Stream(lambda _: [messages()] * 110 + [[CanData(0x252, bytes(8), 0)]])
  (candidate, _), logs = run(stream)
  assert candidate is None
  assert len(logs) == 3


def test_sparse_packets_cannot_meet_minimum_on_retry():
  stream = Stream(lambda _: [messages()])

  def slow_recv(wait_for_one=False):
    stream.now += .1
    return [messages()]
  stream.recv = slow_recv
  (candidate, _), _ = run(stream)
  assert candidate is None


def test_unique_fw_or_fixed_match_skips_retries(monkeypatch):
  monkeypatch.delenv('SKIP_FW_QUERY', raising=False)
  monkeypatch.delenv('DISABLE_FW_CACHE', raising=False)
  for fixed in ('', CAR.CHEVROLET_VOLT):
    monkeypatch.setenv('FINGERPRINT', fixed)
    stream = Stream(lambda _: [])
    send, mux, completed = Mock(), Mock(), Mock()
    with patch.object(helpers.time, 'monotonic', stream.clock), \
         patch.object(helpers, 'get_vin', return_value=(0, 0, '1G1RA6S50HU000001')) as vin, \
         patch.object(helpers, 'get_present_ecus', return_value=set()), \
         patch.object(helpers, 'get_fw_versions_ordered', return_value=[]) as fw, \
         patch.object(helpers, 'match_fw_to_car', return_value=(True, {CAR.CHEVROLET_VOLT} if not fixed else set())):
      result = helpers.fingerprint(stream.recv, send, mux, None, retry_can_fingerprint=True, on_can_attempt=completed)
    assert result[0] == CAR.CHEVROLET_VOLT
    assert result[4] == (helpers.CarParams.FingerprintSource.fixed if fixed else helpers.CarParams.FingerprintSource.fw)
    completed.assert_called_once_with(1)
    vin.assert_called_once()
    fw.assert_called_once()
    assert mux.call_count == 2
    send.assert_not_called()


def test_failed_live_queries_are_not_repeated(monkeypatch):
  monkeypatch.delenv('SKIP_FW_QUERY', raising=False)
  monkeypatch.delenv('FINGERPRINT', raising=False)
  stream = Stream(lambda _: [messages() + [CanData(0x552, bytes(8), 0)]])
  send, mux, completed = Mock(), Mock(), Mock()
  with patch.object(helpers.time, 'monotonic', stream.clock), \
       patch.object(helpers, 'get_vin', return_value=(0, 0, '1G1RA6S50HU000001')) as vin, \
       patch.object(helpers, 'get_present_ecus', return_value=set()) as ecus, \
       patch.object(helpers, 'get_fw_versions_ordered', return_value=[]) as fw, \
       patch.object(helpers, 'match_fw_to_car', return_value=(True, set())):
    result = helpers.fingerprint(stream.recv, send, mux, None, retry_can_fingerprint=True, on_can_attempt=completed)
  assert result[0] is None
  assert [c.args[0] for c in completed.call_args_list] == [1, 2, 3]
  for query in (vin, ecus, fw):
    query.assert_called_once()
  assert mux.call_count == 2
  send.assert_not_called()


def test_default_offline_path_has_no_retries(monkeypatch):
  monkeypatch.setenv('SKIP_FW_QUERY', '1')
  monkeypatch.delenv('FINGERPRINT', raising=False)
  with patch.object(helpers, 'can_fingerprint', return_value=(None, {})) as legacy, \
       patch.object(helpers, 'can_fingerprint_with_retries') as live:
    helpers.fingerprint(Mock(), Mock(), Mock(), None)
  legacy.assert_called_once()
  live.assert_not_called()


@pytest.mark.parametrize('model', list(_FINGERPRINTS))
def test_live_normal_fingerprints(model):
  for fingerprint in _FINGERPRINTS[model]:
    packets = [[CanData(a, bytes(n), b) for b in (0, 1) for a, n in fingerprint.items()]]
    stream = Stream(lambda _, packets=packets: packets)
    (candidate, _), logs = run(stream)
    assert candidate == model
    assert len(logs) == 1


def test_cached_vehicle_cannot_override_failed_can(monkeypatch):
  for key in ('SKIP_FW_QUERY', 'DISABLE_FW_CACHE', 'FINGERPRINT'):
    monkeypatch.delenv(key, raising=False)
  cached = Mock(brand='gm', carVin='1G1RA6S50HU000001', carFw=[Mock()])
  stream = Stream(lambda _: [messages() + [CanData(0x252, bytes(8), 0)]])
  with patch.object(helpers.time, 'monotonic', stream.clock), \
       patch.object(helpers, 'get_vin') as vin, \
       patch.object(helpers, 'match_fw_to_car', return_value=(True, set())):
    result = helpers.fingerprint(stream.recv, Mock(), Mock(), cached, retry_can_fingerprint=True)
  assert result[0] is None
  vin.assert_not_called()


def test_exhausted_retries_select_dashcam_mock(monkeypatch):
  monkeypatch.setenv('SKIP_FW_QUERY', '1')
  monkeypatch.delenv('FINGERPRINT', raising=False)
  stream = Stream(lambda _: [messages() + [CanData(0x252, bytes(8), 0)]])
  completed, send = [], Mock()
  with patch.object(helpers.time, 'monotonic', stream.clock):
    interface = helpers.get_car(stream.recv, send, Mock(), False, False,
                                retry_can_fingerprint=True, on_can_attempt=completed.append)
  assert interface.CP.brand == 'mock'
  assert interface.CP.dashcamOnly
  assert completed == [1, 2, 3]
  send.assert_not_called()
