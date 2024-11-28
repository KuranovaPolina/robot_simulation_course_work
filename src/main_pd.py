from math import sin
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
  x_ref = sin(time)

  return(x_ref)

  # y_ref = 0

  # return(x_ref, y_ref)

def get_shift(path_x, data_x):
  KP = 1

  x_shift = KP * (path_x - data_x)

  return [x_shift]

  # y_shift = KP * (path_y - data_y) / 10
  # y_shift = 0

  # return [x_shift, y_shift]


def control_func_pd(model, data):
  path_x = path_func(time.time() - start_time)
  data_x = data.body('car').xpos[0]  #x

  x_shift = get_shift(path_x, data_x)

  ctrl4x4 = x_shift[0]

  data.ctrl = get_ctrl(ctrl4x4 = ctrl4x4)

with mujoco.viewer.launch_passive(model, data) as viewer:  
  mujoco.set_mjcb_control(control_func_pd)

  while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()
 