import math

import mujoco
import mujoco_viewer
import time

model = mujoco.MjModel.from_xml_path("/home/griffin/Documents/GitHub/doq_viz/twist.xml")
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
mujoco.mj_step(model,data)
t_start = time.time()
for _ in range(10000):
    if viewer.is_alive:
        t = time.time()-t_start

        for i in range(6, 14):
            data.qpos[i] = 0.5 * math.sin(0.5 * t)


        # data.qpos[13] = 0.5 * math.sin(0.5 * t)


        mujoco.mj_kinematics(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()
