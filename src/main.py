import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
from lxml import etree # lxml is needed to change .xml models in script

model = mujoco.MjModel.from_xml_path("../models/project.xml")
data = mujoco.MjData(model)

start_time = 0

def control_func(model, data):
  if step_start - start_time < 3:
    data.ctrl = [0, 0, 0.0, 0, 0, 0, 0]
  elif step_start - start_time < 4:
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  elif step_start - start_time < 5.5:
    data.ctrl = [0, 0, 0.1, 0.1, 0, 0, 0]
  elif step_start - start_time < 7:
    data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  # elif step_start - start_time < 8:
  #   data.ctrl = [0, 0, 0.1, 0, 0, 0, 0]
  else:
    data.ctrl = [0, 0, 0, 0, 0, 0, 0]

with mujoco.viewer.launch_passive(model, data) as viewer:  
  start_time = time.time()
  step_start = start_time

  # mujoco.set_mjcb_control(control_func)

  while viewer.is_running():
    step_start = time.time()
    x = data.body("car").xpos

    print(step_start - start_time, x)

    mujoco.mj_step(model, data)
    viewer.sync()
 