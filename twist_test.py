import math

import mujoco
import mujoco_viewer
import time

model = mujoco.MjModel.from_xml_path("/home/shane/double_spring_double_mass/twist.xml")
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
mujoco.mj_step(model,data)
t_start = time.time()
for _ in range(10000):
    if viewer.is_alive:
        t = time.time()-t_start
        data.qpos[13] = 0.5 * math.sin(0.5 * t)
        mujoco.mj_kinematics(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()