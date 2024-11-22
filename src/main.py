import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
from lxml import etree # lxml is needed to change .xml models in script
import math

model = mujoco.MjModel.from_xml_path("../models/project_parallells.xml")
data = mujoco.MjData(model)

start_time = 0

def control_func_1(model, data):
  l_time = 0.2
  r_time = l_time * math.pi / 4 + 0.1
  wait_time = 3
  time_part = (step_start - start_time) % (wait_time + 8 * l_time + 6 * r_time)

  if (time_part) < wait_time:
    data.ctrl = [0, 0, 0.0, 0, 0, 0, 0]
  elif (time_part) < (wait_time + l_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + l_time + r_time):
    data.ctrl = [0, 0, 0.1, 0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 2 * l_time + r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 2 * l_time + 2 * r_time):
    data.ctrl = [0, 0, 0.1, 0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 3 * l_time + 2 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 3 * l_time + 3 * r_time):
    data.ctrl = [0, 0, 0.1, 0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 4 * l_time + 3 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 5 * l_time + 3 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 5 * l_time + 4 * r_time):
    data.ctrl = [0, 0, 0.1, -0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 6 * l_time + 4 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 6 * l_time + 5 * r_time):
    data.ctrl = [0, 0, 0.1, -0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 7 * l_time + 5 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 7 * l_time + 6 * r_time):
    data.ctrl = [0, 0, 0.1, -0.7, 0, 0, 0]
  elif (time_part) < (wait_time + 8 * l_time + 6 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  else:
    data.ctrl = [0, 0, 0, 0, 0, 0, 0]
    # data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]

def control_func_2(model, data):
  l_time = 0.2
  r_time = l_time * math.pi / 2
  wait_time = 3
  time_part = (step_start - start_time) % (wait_time + 8 * l_time + 4 * r_time)

  if (time_part) < wait_time:
    data.ctrl = [0, 0, 0.0, 0, 0, 0, 0]
  elif (time_part) < (wait_time + l_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + l_time + r_time):
    data.ctrl = [0, 0, 0.1, 0.9, 0, 0, 0]
  elif (time_part) < (wait_time + 2 * l_time + r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 3 * l_time + r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 3 * l_time + 2 * r_time):
    data.ctrl = [0, 0, 0.1, -0.9, 0, 0, 0]
  elif (time_part) < (wait_time + 4 * l_time + 2 * r_time):
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif (time_part) < (wait_time + 4 * l_time + 12 * r_time):
    data.ctrl = [0, 0, 0.05, -0.2, 0, 0, 0]
  else:
    data.ctrl = [0, 0, 0, 0, 0, 0, 0]
    # data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]

with mujoco.viewer.launch_passive(model, data) as viewer:  
  start_time = time.time()
  step_start = start_time

  mujoco.set_mjcb_control(control_func_2)

  while viewer.is_running():
    step_start = time.time()
    x = data.body("car").xpos

    # print(step_start - start_time, x)

    mujoco.mj_step(model, data)
    viewer.sync()
 