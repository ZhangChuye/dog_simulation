import time

import numpy as np
import pybullet as p
import pybullet_data

client = p.connect(p.GUI)
#client = pb.connect(pb.DIRECT) # non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add search path for loadURDF.

#pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0) # disable rendering during creation.
#pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0) # disable user interface during creation.
#pb.configureDebugVisualizer(pb.COV_ENABLE_TINY_RENDERER, 0) # disable tiny renderer during creation.

p.setGravity(0, 0, -10) # set gravity.

parentPath = "/home/ye/Project/mcr/dog_simulation/urdf/"
planeId = p.loadURDF("plane.urdf") # load plane.urdf.
robotID = p.loadURDF(parentPath+"my_bot.urdf", [0, 0, 0.1]) # load r2d2.urdf.

num_joints = p.getNumJoints(robotID)
# get joint info
joint_infos = []
for i in range(num_joints):
	joint_info = p.getJointInfo(robotID, i)
	print(joint_info)
	joint_infos.append(joint_info)

# maxforce = 10
# velocity = -0.4
# for i in range(len(joint_infos)):
#     if i == 0 or 3 or 6 or 9:
#         p.setJointMotorControl2(bodyUniqueId=robotID,
#                                        jointIndex=joint_infos[i][0],
#                                        controlMode=p.VELOCITY_CONTROL,
#                                        targetVelocity=velocity,
#                                        force=maxforce)

startposition = [0, 0, 0.2]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.resetBasePositionAndOrientation(robotID, startposition, robotStartOrientation)

# set original joint state
p.resetJointState(robotID, 0, -1.57/2)
p.resetJointState(robotID, 3, -1.57/2)
p.resetJointState(robotID, 6, -1.57/2)
p.resetJointState(robotID, 9, -1.57/2)

p.resetJointState(robotID, 1, -1.57/4)
p.resetJointState(robotID, 4, -1.57/4)
p.resetJointState(robotID, 7, -1.57/4)
p.resetJointState(robotID, 10, -1.57/4)

p.resetJointState(robotID, 2, 1.57/2)
p.resetJointState(robotID, 5, 1.57/2)
p.resetJointState(robotID, 8, 1.57/2)
p.resetJointState(robotID, 11, 1.57/2)

while(True):
    p.stepSimulation()
    time.sleep(1./240.)
    p.setJointMotorControl2(robotID,2,p.POSITION_CONTROL,targetVelocity=1,force=1000)
    for i in range(len(joint_infos)):
        joint_info = p.getJointInfo(robotID, i)
        print(joint_info)

pb.disconnect()


