from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.volkswagen.values import CAR, PQ_CARS, CANBUS, BUTTON_STATES, NetworkLocation, TransmissionType, GearShifter
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    self.buttonStatesPrev = BUTTON_STATES.copy()

    if CP.networkLocation == NetworkLocation.fwdCamera:
      self.ext_bus = CANBUS.pt
      self.cp_ext = self.cp
    else:
      self.ext_bus = CANBUS.cam
      self.cp_ext = self.cp_cam

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "volkswagen"
    ret.radarOffCan = True

    # Set global PQ35/PQ46/NMS parameters
    # ret.safetyModel = car.CarParams.SafetyModel.volkswagenPq
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volkswagenPq)]
    ret.enableBsm = 0x3BA in fingerprint[0]

    ret.transmissionType = TransmissionType.manual
    ret.networkLocation = NetworkLocation.gateway


    # Global tuning defaults, can be overridden per-vehicle
    ret.steerActuatorDelay = 0.05
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    tire_stiffness_factor = 1.0  # Let the params learner figure this out
    
    ret.lateralTuning.pid.kfBP = [20/3.6, 50/3.6, 70/3.6, 90/3.6]
    ret.lateralTuning.pid.kpBP = [0., 105*CV.KPH_TO_MS]
    ret.lateralTuning.pid.kiBP = [0., 105*CV.KPH_TO_MS]

    ret.lateralTuning.pid.kfV = [0.00060, 0.00030, 0.00020, 0.00010]
    ret.lateralTuning.pid.kpV = [0.15, 0.19]
    ret.lateralTuning.pid.kiV = [0.05,  0.08]

    # Per-chassis tuning values, override tuning defaults here if desired

    if candidate == CAR.GOLF_MK6:
      ret.mass = 1500 + STD_CARGO_KG
      ret.wheelbase = 2.68
      ret.minSteerSpeed = 0 * CV.KPH_TO_MS

    else:
      raise ValueError(f"unsupported car {candidate}")

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.centerToFront = ret.wheelbase * 0.4
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    buttonEvents = []

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_ext, self.CP.transmissionType)
    ret.canValid = True
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # TODO: add a field for this to carState, car interface code shouldn't write params
    # Update the device metric configuration to match the car at first startup,
    # or if there's been a change.
    #if self.CS.displayMetricUnits != self.displayMetricUnitsPrev:
    #  put_nonblocking("IsMetric", "1" if self.CS.displayMetricUnits else "0")

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic])
    
    if ret.clutchPressed:
      events.add(EventName.buttonEnable)
    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.add(EventName.parkBrake)

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    ret.buttonEvents = buttonEvents

    # update previous car states
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c.enabled, self.CS, self.frame, self.ext_bus, c.actuators,
                         hud_control.visualAlert,
                         hud_control.leftLaneVisible,
                         hud_control.rightLaneVisible,
                         hud_control.leftLaneDepart,
                         hud_control.rightLaneDepart)
    self.frame += 1
    return ret
