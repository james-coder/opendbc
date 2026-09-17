import os
import time
from collections.abc import Callable

from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.structs import CarParams, CarParamsT
from opendbc.car.fingerprints import eliminate_incompatible_cars, all_legacy_fingerprint_cars
from opendbc.car.fw_versions import ObdCallback, get_fw_versions_ordered, get_present_ecus, match_fw_to_car
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import BRANDS
from opendbc.car.vin import get_vin, is_valid_vin, VIN_UNKNOWN

FRAME_FINGERPRINT = 100  # 1s


def load_interfaces(brand_names):
  ret = {}
  for brand_name in brand_names:
    path = f'opendbc.car.{brand_name}'
    CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
    for model_name in brand_names[brand_name]:
      ret[model_name] = CarInterface
  return ret


def _get_interface_names() -> dict[str, list[str]]:
  # returns a dict of brand name and its respective models
  brand_names = {}
  for brand in BRANDS:
    brand_name = brand.__module__.split('.')[-2]
    brand_names[brand_name] = [model.value for model in brand]

  return brand_names


# imports from directory opendbc/car/<name>/
interface_names = _get_interface_names()
interfaces = load_interfaces(interface_names)


def can_fingerprint(can_recv: CanRecvCallable, *, timeout: float | None = None, min_duration: float = 0.,
                    attempt: int = 0) -> tuple[str | None, dict[int, dict]]:
  finger = gen_empty_fingerprint()
  candidate_cars = {i: all_legacy_fingerprint_cars() for i in [0, 1]}  # attempt fingerprint on both bus 0 and 1
  frame = 0
  car_fingerprint = None
  done = False
  started = time.monotonic()
  eliminations = []
  reason = "packet_limit"

  while not done:
    if timeout is not None and time.monotonic() - started >= timeout:
      reason = "timeout"
      break
    # can_recv(wait_for_one=True) may return zero or multiple packets, so we increment frame for each one we receive
    can_packets = can_recv(wait_for_one=True)
    for can_packet in can_packets:
      for can in can_packet:
        # The fingerprint dict is generated for all buses, this way the car interface
        # can use it to detect a (valid) multipanda setup and initialize accordingly
        if can.src < 128:
          if can.src not in finger:
            finger[can.src] = {}
          finger[can.src][can.address] = len(can.dat)

        for b in candidate_cars:
          # Ignore extended messages and VIN query response.
          if can.src == b and can.address < 0x800 and can.address not in (0x7df, 0x7e0, 0x7e8):
            previous = candidate_cars[b]
            candidate_cars[b] = eliminate_incompatible_cars(can, previous)
            if timeout is not None:
              removed = [c for c in previous if c not in candidate_cars[b]]
              if removed:
                # Each model can be removed only once per bus: bounded startup evidence.
                eliminations.append({"bus": b, "address": can.address, "length": len(can.dat), "data": can.dat.hex(),
                                     "elapsed": time.monotonic() - started, "candidates": removed})

      if timeout is not None:
        frame += 1
        continue  # Check the entire received batch before accepting a live match.

      # if we only have one car choice and the time since we got our first
      # message has elapsed, exit
      for b in candidate_cars:
        if len(candidate_cars[b]) == 1 and frame > FRAME_FINGERPRINT:
          # fingerprint done
          car_fingerprint = candidate_cars[b][0]

      # bail if no cars left or we've been waiting for more than 2s
      failed = (all(len(cc) == 0 for cc in candidate_cars.values()) and frame > FRAME_FINGERPRINT) or frame > 200
      succeeded = car_fingerprint is not None
      done = failed or succeeded

      frame += 1

    if timeout is not None:
      elapsed = time.monotonic() - started
      matches = {cc[0] for cc in candidate_cars.values() if len(cc) == 1}
      # A deadline reached while receiving/processing is a failure, not a late acceptance.
      if elapsed >= timeout:
        reason, done = "timeout", True
      elif frame >= FRAME_FINGERPRINT + 2 and elapsed >= min_duration:
        if len(matches) == 1:
          car_fingerprint = next(iter(matches))
          reason, done = "matched", True
        elif len(matches) > 1:
          reason, done = "conflicting_buses", True
        elif all(not cc for cc in candidate_cars.values()):
          reason, done = "eliminated", True
        elif min_duration == 0. and frame >= 202:
          reason, done = "packet_limit", True

  if timeout is not None:
    carlog.warning({"event": "can_fingerprint_attempt", "attempt": attempt, "elapsed": time.monotonic() - started,
                    "reason": reason, "candidate": car_fingerprint, "packets": frame,
                    "remaining": candidate_cars, "eliminations": eliminations, "fingerprints": repr(finger)})
  return car_fingerprint, finger


def can_fingerprint_with_retries(can_recv: CanRecvCallable, *, retry: bool = True,
                                 on_attempt: Callable[[int], None] | None = None) -> tuple[str | None, dict[int, dict]]:
  """Live only; requires a bounded receive callback (card uses a 20 ms socket timeout).

  No transmissions or safety/mux changes. Offline/replay callers retain packet-based timing.
  """
  candidate, finger = can_fingerprint(can_recv, timeout=3., attempt=1)
  if on_attempt is not None:
    on_attempt(1)
  if candidate is not None or not retry:
    return candidate, finger

  deadline = time.monotonic() + 8.
  for attempt in (2, 3):
    settle_until = min(time.monotonic() + 1., deadline)
    while time.monotonic() < settle_until:
      can_recv(wait_for_one=True)
    remaining = deadline - time.monotonic()
    if remaining <= 0.:
      break
    candidate, finger = can_fingerprint(can_recv, timeout=min(3., remaining), min_duration=2., attempt=attempt)
    if on_attempt is not None:
      on_attempt(attempt)
    if candidate is not None:
      break
  return candidate, finger


# **** for use live only ****
def fingerprint(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback,
                cached_params: CarParamsT | None, *, retry_can_fingerprint: bool = False,
                on_can_attempt: Callable[[int], None] | None = None
                ) -> tuple[str | None, dict, str, list[CarParams.CarFw], CarParams.FingerprintSource, bool]:
  fixed_fingerprint = os.environ.get('FINGERPRINT', "")
  skip_fw_query = os.environ.get('SKIP_FW_QUERY', False)
  disable_fw_cache = os.environ.get('DISABLE_FW_CACHE', False)
  ecu_rx_addrs = set()

  start_time = time.monotonic()
  if not skip_fw_query:
    if cached_params is not None and cached_params.brand != "mock" and len(cached_params.carFw) > 0 and \
       cached_params.carVin is not VIN_UNKNOWN and not disable_fw_cache:
      carlog.warning("Using cached CarParams")
      vin_rx_addr, vin_rx_bus, vin = -1, -1, cached_params.carVin
      car_fw = list(cached_params.carFw)
      cached = True
    else:
      carlog.warning("Getting VIN & FW versions")
      # enable OBD multiplexing for VIN query
      # NOTE: this takes ~0.1s and is relied on to allow sendcan subscriber to connect in time
      set_obd_multiplexing(True)
      # VIN query only reliably works through OBDII
      vin_rx_addr, vin_rx_bus, vin = get_vin(can_recv, can_send, (0, 1))
      ecu_rx_addrs = get_present_ecus(can_recv, can_send, set_obd_multiplexing)
      car_fw = get_fw_versions_ordered(can_recv, can_send, set_obd_multiplexing, vin, ecu_rx_addrs)
      cached = False

    exact_fw_match, fw_candidates = match_fw_to_car(car_fw, vin)
  else:
    vin_rx_addr, vin_rx_bus, vin = -1, -1, VIN_UNKNOWN
    exact_fw_match, fw_candidates, car_fw = True, set(), []
    cached = False

  if not is_valid_vin(vin):
    carlog.error({"event": "Malformed VIN", "vin": vin})
    vin = VIN_UNKNOWN
  carlog.warning("VIN %s", vin)

  # disable OBD multiplexing for CAN fingerprinting and potential ECU knockouts
  set_obd_multiplexing(False)

  fw_query_time = time.monotonic() - start_time

  # CAN fingerprint
  # drain CAN socket so we get the latest messages
  can_recv()
  if retry_can_fingerprint:
    car_fingerprint, finger = can_fingerprint_with_retries(can_recv, retry=not fixed_fingerprint and len(fw_candidates) != 1,
                                                        on_attempt=on_can_attempt)
  else:
    car_fingerprint, finger = can_fingerprint(can_recv)

  exact_match = True
  source = CarParams.FingerprintSource.can

  # If FW query returns exactly 1 candidate, use it
  if len(fw_candidates) == 1:
    car_fingerprint = list(fw_candidates)[0]
    source = CarParams.FingerprintSource.fw
    exact_match = exact_fw_match

  if fixed_fingerprint:
    car_fingerprint = fixed_fingerprint
    source = CarParams.FingerprintSource.fixed

  carlog.error({"event": "fingerprinted", "car_fingerprint": str(car_fingerprint), "source": source, "fuzzy": not exact_match,
                "cached": cached, "fw_count": len(car_fw), "ecu_responses": list(ecu_rx_addrs), "vin_rx_addr": vin_rx_addr,
                "vin_rx_bus": vin_rx_bus, "fingerprints": repr(finger), "fw_query_time": fw_query_time})

  return car_fingerprint, finger, vin, car_fw, source, exact_match


def wait_for_volt_object_headers(candidate, fingerprints, can_recv: CanRecvCallable):
  """Live startup only: model recognition may precede Object CAN wake-up.

  Receive callback must be bounded (card uses a 20 ms timeout). Never transmit,
  change safety, or infer radar presence from model identity or cached params.
  Missing headers leave the existing GM dashcam-only decision intact.
  """
  from opendbc.car.gm.values import CAR, CanBus
  from opendbc.car.gm.radar_interface import RADAR_HEADER_MSG, CAMERA_DATA_HEADER_MSG

  if candidate != CAR.CHEVROLET_VOLT:
    return
  bus = CanBus.OBSTACLE
  headers = (RADAR_HEADER_MSG, CAMERA_DATA_HEADER_MSG)
  observed = fingerprints.setdefault(bus, {})
  # Reject malformed header evidence rather than merely checking ID presence.
  for address in headers:
    if address in observed and observed[address] != 8:
      del observed[address]
  if any(address in observed for address in headers):
    return

  started = time.monotonic()
  deadline = started + 5.
  found = False
  while time.monotonic() < deadline:
    packets = can_recv(wait_for_one=True)
    if time.monotonic() >= deadline:
      break  # A callback crossing the deadline cannot supply late evidence.
    for packet in packets:
      for frame in packet:
        if frame.src == bus and frame.address in headers and len(frame.dat) == 8:
          observed[frame.address] = 8
          found = True
    if found:
      break
  carlog.warning({"event": "volt_object_startup_wait", "elapsed": time.monotonic() - started,
                  "result": "header_received" if found else "timeout_dashcam",
                  "headers": [address for address in headers if address in observed]})


def get_car(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, alpha_long_allowed: bool,
            is_release: bool, cached_params: CarParamsT | None = None, *, retry_can_fingerprint: bool = False,
            on_can_attempt: Callable[[int], None] | None = None):
  candidate, fingerprints, vin, car_fw, source, exact_match = fingerprint(
    can_recv, can_send, set_obd_multiplexing, cached_params, retry_can_fingerprint=retry_can_fingerprint, on_can_attempt=on_can_attempt)

  if retry_can_fingerprint:
    wait_for_volt_object_headers(candidate, fingerprints, can_recv)

  if candidate is None:
    carlog.error({"event": "car doesn't match any fingerprints", "fingerprints": repr(fingerprints)})
    candidate = "MOCK"

  CarInterface = interfaces[candidate]
  CP: CarParams = CarInterface.get_params(candidate, fingerprints, car_fw, alpha_long_allowed, is_release, docs=False)
  CP.carVin = vin
  CP.carFw = car_fw
  CP.fingerprintSource = source
  CP.fuzzyFingerprint = not exact_match

  return interfaces[CP.carFingerprint](CP)


def get_demo_car_params():
  platform = MOCK.MOCK
  CarInterface = interfaces[platform]
  CP = CarInterface.get_non_essential_params(platform)
  return CP
