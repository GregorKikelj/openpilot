# flake8: noqa

from collections import defaultdict
from typing import Dict
from cereal import car
from selfdrive.car import dbc_dict
Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
  MQB_LDW_STEP = 10              # LDW_02 message frequency 10Hz on MQB
  PQ_LDW_STEP = 5                # LDW_1 message frequency 20Hz on PQ35/PQ46/NMS
  GRA_ACC_STEP = 3               # GRA_ACC_01 message frequency 33Hz

  GRA_VBP_STEP = 100             # Send ACC virtual button presses once a second
  GRA_VBP_COUNT = 16             # Send VBP messages for ~0.5s (GRA_ACC_STEP * 16)

  # Observed documented MQB limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # Limiting rate-of-change based on real-world testing and Comma's safety
  # requirements for minimum time to lane departure.
  STEER_MAX = 300                # Max heading control assist torque 3.00 Nm
  STEER_DELTA_UP = 4             # Max HCA reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

class CANBUS:
  pt = 0
  cam = 2

class DBC_FILES:
  pq = "vw_golf_mk4"  # Used for all cars with PQ25/PQ35/PQ46/NMS-style CAN messaging
  mqb = "vw_mqb_2010"  # Used for all cars with MQB-style CAN messaging

DBC = defaultdict(lambda: dbc_dict(DBC_FILES.mqb, None))  # type: Dict[str, Dict[str, str]]

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

MQB_LDW_MESSAGES = {
  "none": 0,                            # Nothing to display
  "laneAssistUnavailChime": 1,          # "Lane Assist currently not available." with chime
  "laneAssistUnavailNoSensorChime": 3,  # "Lane Assist not available. No sensor view." with chime
  "laneAssistTakeOverUrgent": 4,        # "Lane Assist: Please Take Over Steering" with urgent beep
  "emergencyAssistUrgent": 6,           # "Emergency Assist: Please Take Over Steering" with urgent beep
  "laneAssistTakeOverChime": 7,         # "Lane Assist: Please Take Over Steering" with chime
  "laneAssistTakeOverSilent": 8,        # "Lane Assist: Please Take Over Steering" silent
  "emergencyAssistChangingLanes": 9,    # "Emergency Assist: Changing lanes..." with urgent beep
  "laneAssistDeactivated": 10,          # "Lane Assist deactivated." silent with persistent icon afterward
}

# Check the 7th and 8th characters of the VIN before adding a new CAR. If the
# chassis code is already listed below, don't add a new CAR, just add to the
# FW_VERSIONS for that existing CAR.
# Exception: SEAT Leon and SEAT Ateca share a chassis code

class CAR:
  ATLAS_MK1 = "VOLKSWAGEN ATLAS 1ST GEN"      # Chassis CA, Mk1 VW Atlas and Atlas Cross Sport
  GOLF_MK6 = "VOLKSWAGEN GOLF 6TH GEN"        # Chassis 1K/5K/AJ, includes Mk6 Golf and variants
  GOLF_MK7 = "VOLKSWAGEN GOLF 7TH GEN"        # Chassis 5G/AU/BA/BE, Mk7 VW Golf and variants
  JETTA_MK7 = "VOLKSWAGEN JETTA 7TH GEN"      # Chassis BU, Mk7 Jetta
  PASSAT_MK8 = "VOLKSWAGEN PASSAT 8TH GEN"    # Chassis 3G, Mk8 Passat and variants
  PASSAT_NMS = "VOLKSWAGEN PASSAT NMS"        # Chassis A3, North America/China/Mideast NMS Passat, incl. facelift
  TCROSS_MK1 = "VOLKSWAGEN T-CROSS 1ST GEN"   # Chassis C1, Mk1 VW T-Cross SWB and LWB variants
  TIGUAN_MK2 = "VOLKSWAGEN TIGUAN 2ND GEN"    # Chassis AD/BW, Mk2 VW Tiguan and variants
  TOURAN_MK2 = "VOLKSWAGEN TOURAN 2ND GEN"    # Chassis 1T, Mk2 VW Touran and variants
  AUDI_A3_MK3 = "AUDI A3 3RD GEN"             # Chassis 8V/FF, Mk3 Audi A3 and variants
  AUDI_Q2_MK1 = "AUDI Q2 1ST GEN"             # Chassis GA, Mk1 Audi Q2 (RoW) and Q2L (China only)
  AUDI_Q3_MK2 = "AUDI Q3 2ND GEN"             # Chassis 8U/F3/FS, Mk2 Audi Q3 and variants
  SEAT_ATECA_MK1 = "SEAT ATECA 1ST GEN"       # Chassis 5F, Mk1 SEAT Ateca and CUPRA Ateca
  SEAT_LEON_MK3 = "SEAT LEON 3RD GEN"         # Chassis 5F, Mk3 SEAT Leon and variants
  SKODA_KODIAQ_MK1 = "SKODA KODIAQ 1ST GEN"   # Chassis NS, Mk1 Skoda Kodiaq
  SKODA_SCALA_MK1 = "SKODA SCALA 1ST GEN"     # Chassis NW, Mk1 Skoda Scala and Skoda Kamiq
  SKODA_SUPERB_MK3 = "SKODA SUPERB 3RD GEN"   # Chassis 3V/NP, Mk3 Skoda Superb and variants
  SKODA_OCTAVIA_MK3 = "SKODA OCTAVIA 3RD GEN" # Chassis NE, Mk3 Skoda Octavia and variants

# All PQ35/PQ46/NMS platform CARs should be on this list

PQ_CARS = [CAR.GOLF_MK6, CAR.PASSAT_NMS]

FINGERPRINTS = {
  CAR.GOLF_MK6: [
    {80: 4, 194: 8, 208: 6, 416: 8, 640: 8, 644: 6, 648: 8, 672: 8, 800: 8, 896: 8, 906: 4, 912: 8, 914: 8, 919: 8, 928: 8, 976: 6, 978: 7, 1056: 8, 1152: 8, 1160: 8, 1164: 8, 1184: 8, 1192: 8, 1306: 8, 1312: 8, 1344: 8, 1360: 8, 1386: 8, 1392: 5, 1394: 1, 1408: 8, 1416: 8, 1420: 8, 1423: 8, 1440: 8, 1488: 8, 1490: 8, 1500: 8, 1504: 8, 1654: 2, 1824: 7, 1827: 7, 2000: 8},
  ],
  CAR.PASSAT_NMS: [
    {80: 4, 194: 8, 208: 6, 416: 8, 428: 8, 640: 8, 648: 8, 800: 8, 870: 8, 872: 8, 896: 8, 906: 4, 912: 8, 914: 8, 928: 8, 978: 7, 1056: 8, 1088: 8, 1152: 8, 1184: 8, 1192: 8, 1312: 8, 1386: 8, 1392: 5, 1394: 1, 1408: 8, 1420: 8, 1440: 8, 1472: 8, 1488: 8, 1500: 8, 1527: 4, 1550: 2, 1652: 8, 1654: 3, 1736: 2, 1792: 8, 1824: 7, 1879: 8, 2000: 8},
    {80: 4, 194: 8, 208: 6, 210: 5, 416: 8, 428: 8, 640: 8, 648: 8, 800: 8, 838: 8, 839: 8, 840: 8, 841: 8, 842: 8, 843: 8, 870: 8, 872: 8, 876: 8, 877: 8, 878: 8, 896: 8, 906: 4, 911: 8, 912: 8, 914: 8, 916: 8, 928: 8, 954: 8, 978: 7, 1056: 8, 1088: 8, 1152: 8, 1175: 8, 1184: 8, 1192: 8, 1312: 8, 1386: 8, 1392: 5, 1394: 1, 1408: 8, 1420: 8, 1440: 8, 1470: 5, 1472: 8, 1488: 8, 1500: 8, 1550: 2, 1650: 3, 1651: 8, 1652: 8, 1654: 2, 1691: 4, 1736: 2, 1824: 7, 2000: 8},
  ],
}

# All supported cars should return FW from the engine, srs, eps, and fwdRadar. Cars
# with a manual trans won't return transmission firmware, but all other cars will.
#
# The 0xF187 SW part number query should return in the form of N[NX][NX] NNN NNN [X[X]],
# where N=number, X=letter, and the trailing two letters are optional. Performance
# tuners sometimes tamper with that field (e.g. 8V0 9C0 BB0 1 from COBB/EQT). Tampered
# ECU SW part numbers are invalid for vehicle ID and compatibility checks. Try to have
# them repaired by the tuner before including them in openpilot.

FW_VERSIONS = {
}
