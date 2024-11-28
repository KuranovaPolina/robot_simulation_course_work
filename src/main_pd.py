from math import cos, sin
from random import randint
from matplotlib.pylab import rand
import mujoco
import mujoco.viewer
import time
import numpy as np

model = mujoco.MjModel.from_xml_path("../models/model2.xml")
data = mujoco.MjData(model)

start_time = time.time()

def get_ctrl(ctrl4x4 = 0, turn_acker = 0, turn = 0):
  return [ctrl4x4, turn_acker, turn]

def path_func(time):
  x_ref = 0.5 * cos(time) 
  y_ref = 0.5 * sin(time) 

  return (x_ref, y_ref)

def get_diff(x_ref, x_real, y_ref, y_real):
  KP_x = 1
  KP_y = 1

  x_diff = KP_x * (x_ref - x_real) 
  y_diff = KP_y * (y_ref - y_real) 

  return (x_diff, y_diff)


def control_func_pd(model, data):
  x_ref, y_ref = path_func(time.time() - start_time)
  x_real = data.body('car').xpos[0]  #x
  y_real = data.body('car').xpos[1]  #y

  x_diff, y_diff = get_diff(x_ref, x_real, y_ref, y_real)

  ctrl4x4 = x_diff

  if y_diff >= 100:
    turn_acker = 0.99
  elif y_diff <= -100:
    turn_acker = -0.99
  else:
    turn_acker = y_diff / 100

  data.ctrl = get_ctrl(ctrl4x4 = ctrl4x4, turn_acker = turn_acker)

with mujoco.viewer.launch_passive(model, data) as viewer:  
  mujoco.set_mjcb_control(control_func_pd)

  while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()
 