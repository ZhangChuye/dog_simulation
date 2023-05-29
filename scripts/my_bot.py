import time

import pybullet as pb
import pybullet_data

client = pb.connect(pb.GUI)
# client = pb.connect(pb.DIRECT) # non-graphical version
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) # add search path for loadURDF.

#pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0) # disable rendering during creation.
#pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0) # disable user interface during creation.
#pb.configureDebugVisualizer(pb.COV_ENABLE_TINY_RENDERER, 0) # disable tiny renderer during creation.

pb.setGravity(0, 0, -10) # set gravity.

parentPath = "/home/ye/Project/mcr/dog_simulation/urdf/"
planeId = pb.loadURDF("plane.urdf") # load plane.urdf.
robotID = pb.loadURDF(parentPath+"my_bot.urdf", [0, 0, 1]) # load r2d2.urdf.

while(True):
    pb.stepSimulation()
    time.sleep(1./240.)

pb.disconnect()
