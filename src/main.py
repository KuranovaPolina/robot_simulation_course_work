import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import time
from lxml import etree # lxml is needed to change .xml models in script

# model = mujoco.MjModel.from_xml_path("/Users/polinakuranova/uni/robot_simulation/robot_simulation/course_work/models/differentialdrive.xml")
model = mujoco.MjModel.from_xml_path("/Users/polinakuranova/uni/robot_simulation/robot_simulation/course_work/models/car.xml")
data = mujoco.MjData(model)

# print(data.ctrl)

with mujoco.viewer.launch_passive(model, data) as viewer:
  data.ctrl = [50., 0.1]
  while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()
 