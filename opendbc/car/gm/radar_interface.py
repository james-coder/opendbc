#!/usr/bin/env python3
import math
from enum import IntFlag
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.values import DBC, CanBus
from opendbc.car.interfaces import RadarInterfaceBase

RADAR_HEADER_MSG = 1120  # F_LRR_Obj_Header
CAMERA_DATA_HEADER_MSG = 1056  # F_Vision_Obj_Header
SLOT_1_MSG = RADAR_HEADER_MSG + 1
NUM_SLOTS = 20

# Actually it's 0x47f, but can parser only reports
# messages that are present in DBC
LAST_RADAR_MSG = RADAR_HEADER_MSG + NUM_SLOTS


class GMRadarFaultBit(IntFlag):
  SENSOR_BLOCKED = 1 << 0          # FLRRSnsrBlckd
  SENSITIVITY = 1 << 1             # FLRRSnstvFltPrsntInt
  YAW_RATE_PLAUSIBILITY = 1 << 2   # FLRRYawRtPlsblityFlt
  HARDWARE = 1 << 3                # FLRRHWFltPrsntInt
  ANTENNA_TUNING = 1 << 4          # FLRRAntTngFltPrsnt
  ALIGNMENT = 1 << 5               # FLRRAlgnFltPrsnt


def create_radar_can_parser(car_fingerprint):
  # C1A-ARS3-A by Continental
  radar_targets = list(range(SLOT_1_MSG, SLOT_1_MSG + NUM_SLOTS))
  signals = list(zip(['FLRRNumValidTargets',
                      'FLRRSnsrBlckd', 'FLRRYawRtPlsblityFlt',
                      'FLRRHWFltPrsntInt', 'FLRRAntTngFltPrsnt',
                      'FLRRAlgnFltPrsnt', 'FLRRSnstvFltPrsntInt'] +
                     ['TrkRange'] * NUM_SLOTS + ['TrkRangeRate'] * NUM_SLOTS +
                     ['TrkRangeAccel'] * NUM_SLOTS + ['TrkAzimuth'] * NUM_SLOTS +
                     ['TrkWidth'] * NUM_SLOTS + ['TrkObjectID'] * NUM_SLOTS,
                     [RADAR_HEADER_MSG] * 7 + radar_targets * 6, strict=True))

  messages = list({(s[1], 14) for s in signals})

  return CANParser(DBC[car_fingerprint][Bus.radar], messages, CanBus.OBSTACLE)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.rcp = None if CP.radarUnavailable else create_radar_can_parser(CP.carFingerprint)

    self.trigger_msg = LAST_RADAR_MSG
    self.updated_messages = set()

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    header = self.rcp.vl[RADAR_HEADER_MSG]

    fault_bits = GMRadarFaultBit(0)
    if header['FLRRSnsrBlckd']:
      fault_bits |= GMRadarFaultBit.SENSOR_BLOCKED
    if header['FLRRSnstvFltPrsntInt']:
      fault_bits |= GMRadarFaultBit.SENSITIVITY
    if header['FLRRYawRtPlsblityFlt']:
      fault_bits |= GMRadarFaultBit.YAW_RATE_PLAUSIBILITY
    if header['FLRRHWFltPrsntInt']:
      fault_bits |= GMRadarFaultBit.HARDWARE
    if header['FLRRAntTngFltPrsnt']:
      fault_bits |= GMRadarFaultBit.ANTENNA_TUNING
    if header['FLRRAlgnFltPrsnt']:
      fault_bits |= GMRadarFaultBit.ALIGNMENT

    if not self.rcp.can_valid:
      ret.errors.canError = True
    if fault_bits:
      # Radar self-reports a recoverable fault (e.g. blocked by dirt/rain, temporary
      # misalignment). Don't disable the car - ignore its points this cycle and let
      # the vision-based lead tracking in radard.py take over until it clears on its own.
      ret.errors.radarDegraded = True
      ret.errors.radarDegradedReasons = int(fault_bits)

    currentTargets = set()
    num_targets = 0 if fault_bits else header['FLRRNumValidTargets']

    # Not all radar messages describe targets,
    # no need to monitor all of the self.rcp.msgs_upd
    for ii in self.updated_messages:
      if ii == RADAR_HEADER_MSG:
        continue

      if num_targets == 0:
        break

      cpt = self.rcp.vl[ii]
      # Zero distance means it's an empty target slot
      if cpt['TrkRange'] > 0.0:
        targetId = cpt['TrkObjectID']
        currentTargets.add(targetId)
        if targetId not in self.pts:
          self.pts[targetId] = structs.RadarData.RadarPoint()
          self.pts[targetId].trackId = targetId
        distance = cpt['TrkRange']
        self.pts[targetId].dRel = distance  # from front of car
        # From driver's pov, left is positive
        self.pts[targetId].yRel = math.sin(cpt['TrkAzimuth'] * CV.DEG_TO_RAD) * distance
        self.pts[targetId].vRel = cpt['TrkRangeRate']
        self.pts[targetId].aRel = float('nan')
        self.pts[targetId].yvRel = float('nan')

    for oldTarget in list(self.pts.keys()):
      if oldTarget not in currentTargets:
        del self.pts[oldTarget]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
