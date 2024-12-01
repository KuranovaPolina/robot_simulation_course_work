from math import cos, sin
from random import randint
from matplotlib.pylab import rand
import mujoco
import mujoco.viewer
import time
import numpy as np

import json

model = mujoco.MjModel.from_xml_path("../models/model_acker_final.xml")
data = mujoco.MjData(model)

start_time = time.time()

def get_ctrl(ctrl4x4 = 0, turn_acker = 0, turn = 0):
  return [ctrl4x4, turn_acker, turn, 0, 0]

def path_func(time):
  x_ref = 0.5 * cos(time) + 0.5
  y_ref = 0.5 * sin(time) - 0.5

  # x_ref = cos(time) 
  # y_ref = 0

  # x_ref = 0
  # y_ref = cos(time)

  return (x_ref, y_ref)

def get_diff(x_ref, x_real, y_ref, y_real, v_x_real, v_y_real):
  KP_x = 1
  KP_y = 1
  KP_v_x = 1
  KP_v_y = 1

  x_diff = KP_x * (x_ref - x_real) + KP_v_x * (0 - v_x_real)
  y_diff = KP_y * (y_ref - y_real) + KP_v_y * (0 - v_y_real)

  return (x_diff, y_diff)


def control_func_pd(model, data):
  x_ref, y_ref = path_func(time.time() - start_time)
  x_real = data.body('car').xpos[0]  #x
  y_real = data.body('car').xpos[1]  #y
  v_x_real = data.qvel[0]  #x
  v_y_real = data.qvel[1]  #y

  x_diff, y_diff = get_diff(x_ref, x_real, y_ref, y_real, v_x_real, v_y_real)

  ctrl4x4 = x_diff

  if y_diff >= 1:
    turn_acker = 0.99
  elif y_diff <= -1:
    turn_acker = -0.99
  else:  
    turn_acker = y_diff

  data.ctrl = get_ctrl(ctrl4x4 = ctrl4x4 / 100, turn_acker = turn_acker)


x_values = [ ]
y_values = [ ]

circle_left_values = [ ]
circle_right_values = [ ]

time_values = [ ]

with mujoco.viewer.launch_passive(model, data) as viewer:  
  mujoco.set_mjcb_control(control_func_pd)

  while viewer.is_running() and (time.time() - start_time) < 50:
    x_values.append(data.body('car').xpos[0])
    y_values.append(data.body('car').xpos[1])

    # print(data.body('car').xpos[0], data.qpos[0], data.body('car').xpos[1], data.qpos[1], data.qvel)

#TODO - change x to angle
    circle_left_values.append(data.joint('front left').qpos[0])
    circle_right_values.append(data.joint('front right').qpos[0])

    # print(data.qpos)

    # left_joint_index = data.joint('front left')
    # right_joint_index = data.joint('front right')

    # print(left_joint_index.qpos[0])
    # print(right_joint_index.qpos[0])

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
 