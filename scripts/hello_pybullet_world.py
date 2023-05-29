# sudo pip3 install pybullet
import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotID = p.loadURDF("/home/ye/Project/mcr/dog_simulation/urdf/my_bot.urdf", [0, 0, 1]) # load r2d2.urdf.



for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

print(cubePos,cubeOrn)
p.disconnect()

