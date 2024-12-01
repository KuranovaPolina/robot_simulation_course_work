from math import atan, cos, pi, sin
from random import randint
from matplotlib.pylab import rand
import mujoco
import mujoco.viewer
import time
import numpy as np

import json

model = mujoco.MjModel.from_xml_path("/Users/polinakuranova/uni/robot_simulation/robot_simulation_course_work/robot_simulation_course_work/models/model_acker_final.xml")
data = mujoco.MjData(model)

start_time = time.time()

def get_ctrl(ctrl4x4 = 0, turn_acker = 0, turn = 0):
  return [ctrl4x4, turn_acker, 0, 0]

def path_func(time):
  X = 0.045 # T
  Y = 0.07  # L
  R = 0.5

  left_angle = atan(Y / (R - X / 2))
  right_angle = atan(Y / (R + X / 2))
  ctrl4x4 = 0.1

  return (left_angle, right_angle, ctrl4x4)

def get_diff(data, left_angle_ref, right_angle_ref, ctrl4x4_ref):
  left_angle_diff = 1 * (left_angle_ref - data.joint('front left').qpos[0])
  right_angle_diff = 1 * (right_angle_ref - data.joint('front right').qpos[0])
  ctrl4x4_diff = 1 * (ctrl4x4_ref - data.ctrl[0])

  return (left_angle_diff, right_angle_diff, ctrl4x4_diff)


def control_func_pd(model, data):
  left_angle, right_angle, ctrl4x4 = path_func(time.time() - start_time)

  left_angle_diff, right_angle_diff, ctrl4x4_diff = get_diff(data, left_angle, right_angle, ctrl4x4)

  data.joint('front left').qpos[0] += left_angle_diff
  data.joint('front right').qpos[0] += right_angle_diff
  data.ctrl[0] += ctrl4x4_diff

x_values = [ ]
y_values = [ ]

circle_left_values = [ ]
circle_right_values = [ ]

time_values = [ ]

with mujoco.viewer.launch_passive(model, data) as viewer:  
  mujoco.set_mjcb_control(control_func_pd)

  while viewer.is_running() and (time.time() - start_time) < 120:
    x_values.append(data.body('car').xpos[0])
    y_values.append(data.body('car').xpos[1])

    circle_left_values.append(data.joint('front left').qpos[0])
    circle_right_values.append(data.joint('front right').qpos[0])

    time_values.append(time.time() - start_time)

    mujoco.mj_step(model, data)
    viewer.sync()

  with open('x_values.txt', 'w') as fw:
    json.dump(x_values, fw)

  with open('y_values.txt', 'w') as fw:
    json.dump(y_values, fw)

  with open('circle_left_values.txt', 'w') as fw:
    json.dump(circle_left_values, fw)

  with open('circle_right_values.txt', 'w') as fw:
    json.dump(circle_right_values, fw)

  with open('time_values.txt', 'w') as fw:
    json.dump(time_values, fw)
 