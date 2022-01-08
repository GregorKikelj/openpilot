import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.buttonStates = BUTTON_STATES.copy()

    can_define = CANDefine(DBC_FILES.pq)
    self.get_can_parser = self.get_pq_can_parser
    self.get_cam_can_parser = self.get_pq_cam_can_parser
    self.update = self.update_pq
    self.hca_status_values = can_define.dv["Lenkhilfe_2"]["LH2_Sta_HCA"]

  def update_pq(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    fan = pt_cp.vl["Klima_1"]["Geblaeselast_4_1"]
    print(fan)
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["Bremse_3"]["Radgeschw__VL_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["Bremse_3"]["Radgeschw__VR_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["Bremse_3"]["Radgeschw__HL_4_1"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["Bremse_3"]["Radgeschw__HR_4_1"] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]["Giergeschwindigkeit"] * (1, -1)[int(pt_cp.vl["Bremse_5"]["Vorzeichen_der_Giergeschwindigk"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerError = hca_status in ["DISABLED", "FAULT"]
    ret.steerWarning = hca_status in ["INITIALIZING", "REJECTED"]

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_3"]["Fahrpedal_Rohsignal"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["Bremse_5"]["Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremstestschalter"])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]["Kupplungsschalter"]
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    # TODO: read PQ Einheiten here
    self.displayMetricUnits = True

    # Consume blind-spot monitoring info/warning LED states, if available. The
    # info signal (LED on) is enabled whenever a vehicle is detected in the
    # driver's blind spot. The warning signal (LED flashing) is enabled if the
    # driver shows possibly hazardous intent toward a BSM detected vehicle, by
    # setting the turn signal in that direction, or (for cars with factory Lane
    # Assist) approaches the lane boundary in that direction. Size of the BSM
    # detection box is dynamic based on speed and road curvature.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assist Systems,
    # pages 32-35.
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # TODO: Consume lane departure data from factory camera, if present
    self.ldw_lane_warning_left = False
    self.ldw_lane_warning_right = False
    self.ldw_side_dlc_tlc = None
    self.ldw_dlc = None
    self.ldw_tlc = None
    # TODO: Consume FCW/AEB data from factory radar, if present

    # Update ACC radar status.
    accStatus = ext_cp.vl["ACC_GRA_Anziege"]["ACA_StaACC"]
    if accStatus == 2:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif accStatus in [3, 4, 5]:
      # ACC okay and enabled, currently engaged and regulating speed (3) or engaged with driver accelerating (4) or overrun (5)
      # Verify against Motor_2 to keep in lockstep with Panda safety
      ret.cruiseState.available = True
      if pt_cp.vl["Motor_2"]["GRA_Status"] in [1, 2]:
        ret.cruiseState.enabled = True
      else:
        ret.cruiseState.enabled = False
    else:
      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anziege"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Up_kurz"]) or bool(pt_cp.vl["GRA_Neu"]["GRA_Up_lang"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Down_kurz"]) or bool(pt_cp.vl["GRA_Neu"]["GRA_Down_lang"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Neu_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Recall"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_Neu"]["GRA_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_Neu"]["GRA_Hauptschalt"]
    self.graSenderCoding = pt_cp.vl["GRA_Neu"]["GRA_Sender"]
    self.graTypHauptschalter = False
    self.graButtonTypeInfo = False
    self.graTipStufe2 = False

    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_Neu"]["GRA_Neu_Zaehler"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    return ret

  @staticmethod
  def get_pq_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("LH3_BLW", "Lenkhilfe_3", 0),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3", 0),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3", 0),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3", 0),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2", 0),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1", 0),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1", 0),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3", 0),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3", 0),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3", 0),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3", 0),         # ABS wheel speed, rear right
      ("Giergeschwindigkeit", "Bremse_5", 0),       # Absolute yaw rate
      ("Vorzeichen_der_Giergeschwindigk", "Bremse_5", 0),  # Yaw rate sign
      ("Gurtschalter_Fahrer", "Airbag_1", 0),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1", 0),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2", 0),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2", 0),         # Brakes applied (brake light switch)
      ("Bremsdruck", "Bremse_5", 0),                # Brake pressure applied
      ("Vorzeichen_Bremsdruck", "Bremse_5", 0),     # Brake pressure applied sign (???)
      ("Fahrpedal_Rohsignal", "Motor_3", 0),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1", 0),       # Stability control disabled
      ("GRA_Status", "Motor_2", 0),                 # ACC engagement status
      ("GK1_Fa_Tuerkont", "Gate_Komf_1", 0),        # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1", 0),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1", 0),         # Right turn signal on
      ("Bremsinfo", "Kombi_1", 0),                  # Manual handbrake applied
      ("GRA_Hauptschalt", "GRA_Neu", 0),            # ACC button, on/off
      ("GRA_Abbrechen", "GRA_Neu", 0),              # ACC button, cancel
      ("GRA_Neu_Setzen", "GRA_Neu", 0),             # ACC button, set
      ("GRA_Up_lang", "GRA_Neu", 0),                # ACC button, increase or accel, long press
      ("GRA_Down_lang", "GRA_Neu", 0),              # ACC button, decrease or decel, long press
      ("GRA_Up_kurz", "GRA_Neu", 0),                # ACC button, increase or accel, short press
      ("GRA_Down_kurz", "GRA_Neu", 0),              # ACC button, decrease or decel, short press
      ("GRA_Recall", "GRA_Neu", 0),                 # ACC button, resume
      ("GRA_Zeitluecke", "GRA_Neu", 0),             # ACC button, time gap adj
      ("GRA_Neu_Zaehler", "GRA_Neu", 0),            # ACC button, time gap adj
      ("GRA_Sender", "GRA_Neu", 0),                 # GRA Sender Coding
      ("Geblaeselast_4_1", "Klima_1", 0),           # AC blower fan, because there's no CC
    ]

    checks = [
      # sig_address, frequency
      # ("Bremse_1", 100),          # From J104 ABS/ESP controller
      # ("Bremse_3", 100),          # From J104 ABS/ESP controller
      # ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      # ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      # ("Motor_3", 100),           # From J623 Engine control module
      # ("Airbag_1", 50),           # From J234 Airbag control module
      # ("Bremse_5", 50),           # From J104 ABS/ESP controller
      # ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      # ("Kombi_1", 50),            # From J285 Instrument cluster
      # ("Motor_2", 50),            # From J623 Engine control module
      # ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      # ("Gate_Komf_1", 10),        # From J533 CAN gateway
    ]

    signals += [("Kupplungsschalter", "Motor_1", 0),  # Clutch switch
                ("GK1_Rueckfahr", "Gate_Komf_1", 0)]  # Reverse light from BCM

    return CANParser(DBC_FILES.pq, signals, checks, CANBUS.pt, enforce_checks=False)

  @staticmethod
  def get_pq_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("Kombi_Lamp_Green", "LDW_1", 0),               # Just to check camera for CAN bus validity
    ]

    checks = [
      # sig_address, frequency
      #("LDW_1", 20)        # From R242 Driver assistance camera
    ]

    if CP.networkLocation == NetworkLocation.gateway:
      # Extended CAN devices other than the camera are here on CANBUS.cam
      signals += PqExtraSignals.fwd_radar_signals
      checks += PqExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += PqExtraSignals.bsm_radar_signals
        checks += PqExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.pq, signals, checks, CANBUS.cam, enforce_checks=False)

class PqExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACA_StaACC", "ACC_GRA_Anziege", 0),           # ACC drivetrain coordinator status
    ("ACA_V_Wunsch", "ACC_GRA_Anziege", 0),         # ACC set speed
  ]
  fwd_radar_checks = [
    ("ACC_GRA_Anziege", 25),                        # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_1", 0),           # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_1", 0),             # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_1", 0),           # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_1", 0),             # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_1", 20),                                  # From J1086 Lane Change Assist
  ]
