import math

from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import log, messaging
import numpy as np
from selfdrive.car.volkswagen.values import CarControllerParams as P

from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from common.numpy_fast import clip
from random import random

MODEL_MIN_SPEED = 90 / 3.6 # minimum speed to use model

STEER_FACTOR = 300 

#steering angle, speed, torque, IMU_linear, IMU_angular
norm = (18.600000381469727, 33.88642883300781, 1.0, [18.733535766601562, 8.428146362304688, 9.84222412109375], [0.103515625, 0.3721466064453125, 0.1421051025390625])
groups = [2, 10, 25, 10, 10]

#Data storage timings
prev_data = 100
fwd_data = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

try:
  wb = np.load('/data/openpilot/model_s2_weights.npz', allow_pickle=True)
except:
  wb = np.load('/home/gregor/openpilot/model_s2_weights.npz', allow_pickle=True)
w, b = wb['wb']

def model(x):
  x = np.array(x, dtype=np.float32)
  l0 = np.dot(x, w[0]) + b[0]
  l0 = np.maximum(0, l0)
  l1 = np.dot(l0, w[1]) + b[1]
  l1 = np.maximum(0, l1)
  l2 = np.dot(l1, w[2]) + b[2]
  l2 = np.maximum(0, l2)
  l3 = np.dot(l2, w[3]) + b[3]
  l3 = np.tanh(l3)
  return l3

def get_model_input(phi, v, M, IMU_v, IMU_alpha, M_fut):
  phi_out = phi.reshape(-1, groups[0]).mean(axis=1)/norm[0]
  v_out = v.reshape(-1, groups[1]).mean(axis=1)/norm[1]
  M_out_l = M.reshape(-1, groups[2]).mean(axis=1)/norm[2]
  M_out_r = M_fut.reshape(-1, groups[2]).mean(axis=1)/norm[2]
  M_out = np.concatenate([M_out_l, M_out_r])
  IMU_v_out = [IMU_v[i].reshape(-1, groups[3]).mean(axis=1)/norm[3][i] for i in range(3)]
  IMU_alpha_out = [IMU_alpha[i].reshape(-1, groups[4]).mean(axis=1)/norm[4][i] for i in range(3)]

  return np.concatenate([phi_out, v_out, M_out, IMU_v_out[0], IMU_v_out[1], IMU_v_out[2], IMU_alpha_out[0], IMU_alpha_out[1], IMU_alpha_out[2]])


def apply_std_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS) -> int:

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + (LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.STEER_MAX + (-LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque))) # int sems unneded

class ModelControls:
  def __init__(self):
    self.phi = np.zeros((prev_data,))
    self.v = np.zeros((prev_data,))
    self.M = np.zeros((prev_data,))
    self.IMU_v = [np.zeros((prev_data,)) for _ in range(3)]
    self.IMU_alpha = [np.zeros((prev_data,)) for _ in range(3)]

    self.active = False #Ensure good init on change
    self.outputTorque = 0 

  def parse_logs(self, phi, v, M, IMU_v, IMU_alpha):

    if not self.active and v > MODEL_MIN_SPEED:
      self.active = True
    if self.active and v < MODEL_MIN_SPEED - 5/3.6:
      self.active = False

    # spravi phi, v, M v numPy array
    self.phi = np.roll(self.phi, -1)
    self.phi[-1]=phi
    self.v = np.roll(self.v, -1)
    self.v[-1]=v
    self.M = np.roll(self.M, -1)
    self.M[-1]=M
    for i in range(3):
      self.IMU_v[i] = np.roll(self.IMU_v[i], -1)
      self.IMU_v[i][-1] = IMU_v[i]
      self.IMU_alpha[i] = np.roll(self.IMU_alpha[i], -1)
      self.IMU_alpha[i][-1] = IMU_alpha[i]
    
  def predict_torque(self, lat_plan, CP, VM, angle_offset, CS):
    # Compute desired steering angle profile 
    steering_angle = [0]*len(fwd_data)
    for i in range(len(fwd_data)):
      CP.steerActuatorDelay = -0.2 + fwd_data[i]/100
      desired_curvature, _ = get_lag_adjusted_curvature(CP, self.v[-1], lat_plan.psis, lat_plan.curvatures, lat_plan.curvatureRates)

      steering_angle[i] = math.degrees(VM.get_steer_from_curvature(-desired_curvature, self.v[-1])) + angle_offset

    # TODO: consider adding
    # steering_angle = [CS.steeringAngleDeg] + steering_angle

    M_fut = np.ones((fwd_data[-1],))*self.M[-1]

    model_input = get_model_input(self.phi, self.v, self.M, self.IMU_v, self.IMU_alpha, M_fut)
    predicted_angle = model(model_input) * norm[0]

    # TODO: consider adding
    # CP.steerActuatorDelay = -0.2
    # predicted_angle = [get_lag_adjusted_curvature(CP, self.v[-1], lat_plan.psis, lat_plan.curvatures, lat_plan.curvatureRates)] + predicted_angle

    #L1 je negativen ce moramo precej bolj zaviti
    a = 1
    w = [a/(a+i) for i in range(len(predicted_angle))]
    L1 = sum([(predicted_angle[i]-steering_angle[i])*w[i] for i in range(len(predicted_angle))])

    #Poenostavljena logika: Ali je trenutni navor dovolj velik?
    if L1 > 0:
      self.outputTorque = apply_std_steer_torque_limits(-STEER_FACTOR, STEER_FACTOR*self.outputTorque, CS.steeringTorque, P)/STEER_FACTOR
    else:
      self.outputTorque = apply_std_steer_torque_limits(STEER_FACTOR, STEER_FACTOR*self.outputTorque, CS.steeringTorque, P)/STEER_FACTOR
    if random() < 0.01:
      print("Right" if L1 > 0 else "Left")


# Create our controller
class LatControlPID():
  def __init__(self, CP, CI):
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            (CP.lateralTuning.pid.kfBP, CP.lateralTuning.pid.kfV),
                            pos_limit=1.0, neg_limit=-1.0,
                            sat_limit=CP.steerLimitTimer)
    self.count = 0
    self.model = ModelControls()
    self.sensor = messaging.sub_sock('sensorEvents')
    self.M = 0
    self.IMU_v = [0]*3
    self.IMU_alpha = [0]*3
    self.CP_actuatorDelay = CP.steerActuatorDelay

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, last_actuators, desired_curvature, lat_plan):
    self.count+=1
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    if CS.vEgo < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      # TODO: feedforward something based on lat_plan.rateSteers
      steer_feedforward = angle_steers_des_no_offset  # offset does not contribute to resistive torque
      steer_feedforward *= CS.vEgo**2  # proportional to realigning tire momentum (~ lateral accel)

      deadzone = 0.15

      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(angle_steers_des, CS.steeringAngleDeg, check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.saturated = bool(self.pid.saturated)
      pid_log.angleError = 1 if self.model.active else 0

      #Parse the model info
      sensors = messaging.recv_sock(self.sensor)
      if sensors is not None:
        for sensor in sensors.sensorEvents:
          if sensor.type == 1: # accelerometer
            self.IMU_v = sensor.acceleration.v
          if sensor.type == 4:  # gyro
            self.IMU_alpha = sensor.gyro.v
      self.model.parse_logs(CS.steeringAngleDeg, CS.vEgo, self.M, self.IMU_v, self.IMU_alpha)

      if self.count % 2 == 0:
        self.model.predict_torque(lat_plan, CP, VM, params.angleOffsetDeg, CS)
        CP.steerActuatorDelay = self.CP_actuatorDelay

      if self.model.active:
        output_steer = self.model.outputTorque
        pid_log.p = 0
        pid_log.i = 0
        pid_log.f = angle_steers_des # desired angle, good for plotting
      #/Parse the model info
      
      pid_log.output = output_steer
      self.M = output_steer
      # print(self.model.active, self.M)

    return output_steer, angle_steers_des, pid_log