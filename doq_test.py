import math

import mujoco
import mujoco_viewer
import time

model = mujoco.MjModel.from_xml_path("/home/griffin/Documents/mujoco/doq_vis/doq.xml")
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
mujoco.mj_step(model,data)
t_start = time.time()

data.qpos[2] = 0.5
for _ in range(100000):
    if viewer.is_alive:
        t = time.time()-t_start

        mujoco.mj_step(model,data)
        # for i in range(7, 14):
        #     data.qpos[i] = 0.5 * math.sin(0.5 * t)


        # data.qpos[13] = 0.5 * math.sin(0.5 * t)



        mujoco.mj_kinematics(model, data)
        viewer.render()
    else:
        break

mujoco.mj_deleteData(data)
mujoco.mj_deleteModel(model)

# close
# viewer.close()
