import time

import numpy as np
import pybullet as p
import pybullet_data

client = p.connect(p.GUI)
# client = pb.connect(pb.DIRECT) # non-graphical version

# add search path for loadURDF.
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0) # disable rendering during creation.
# pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0) # disable user interface during creation.
# pb.configureDebugVisualizer(pb.COV_ENABLE_TINY_RENDERER, 0) # disable tiny renderer during creation.

p.setGravity(0, 0, -10)  # set gravity.

# load the urdf file
parentPath = "/home/ye/Project/mcr/dog_simulation/urdf/"
planeId = p.loadURDF("plane.urdf")  # load plane.urdf.
robotID = p.loadURDF(parentPath+"my_bot.urdf", [0, 0, 0.1])

# set original position
startposition = [0, 0, 0.2]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
p.resetBasePositionAndOrientation(
    robotID, startposition, robotStartOrientation)

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

num_joints = p.getNumJoints(robotID)
joint_infos = []
joint_pos = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robotID, i)
    # print(joint_info)
    joint_infos.append(joint_info)

    link_state = p.getJointState(robotID, i)
    link_position = link_state[0]  # position of link's origin
    link_orientation = link_state[1]  # orientation of link's origin
    print("Link", i, "position:", link_position)
    joint_pos.append(link_position)


Joint2StartPos = joint_pos[4]
Joint2EndPos = [0, 0, 0]
Joint2EndOrientation = p.getQuaternionFromEuler([0, 0, -1])

starPos_array = np.array(Joint2StartPos)
endPos_array = np.array(Joint2EndPos)

step = 5
step_array = (endPos_array-starPos_array)/step

# p.setRealTimeSimulation(0)
flag = False
while (True):
    if flag:
        continue
    for i in range(step):
        print("step:", i)
        Joint2StepPos = list(starPos_array + step_array)
        targetJointPos = p.calculateInverseKinematics(robotID, 2, Joint2StepPos, Joint2EndOrientation)
        print("targetJointPos:", targetJointPos)
        p.setJointMotorControlArray(robotID, range(num_joints), p.POSITION_CONTROL, targetPositions=targetJointPos)
        p.stepSimulation()
    if i == step:
        flag = True

# while (True):
#     p.stepSimulation()
#     time.sleep(1./240.)
#     p.setJointMotorControl2(robotID, 2, p.VELOCITY_CONTROL,
#                             targetVelocity=1, force=1000)
#     for i in range(len(joint_infos)):
#         joint_info = p.getJointInfo(robotID, i)
#         # print(joint_info)

p.disconnect()