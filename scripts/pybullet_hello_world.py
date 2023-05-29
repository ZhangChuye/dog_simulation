import time

import numpy as np
import pybullet
import pybullet_data


if __name__ == '__main__':
    client = pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setPhysicsEngineParameter(numSolverIterations=10)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setRealTimeSimulation(1)

    shift = [0, 0, 0]
    scale = [1, 1, 1]

    visual_shape_id = pybullet.createVisualShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
        visualFramePosition=[0, 0, 0],
        meshScale=scale)
    collision_shape_id = pybullet.createCollisionShape(
        shapeType=pybullet.GEOM_MESH,
        fileName="sphere_smooth.obj",
        collisionFramePosition=[0, 0, 0],
        meshScale=scale)
    pybullet.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-2, -1, 1],
        useMaximalCoordinates=True)

    plane_id = pybullet.loadURDF("plane100.urdf", useMaximalCoordinates=True)
    cube_ind = pybullet.loadURDF('cube.urdf', (3, 1, 1), pybullet.getQuaternionFromEuler([0, 0, 0]))
    r_ind = pybullet.loadURDF('r2d2.urdf', (1, 1, 1),  pybullet.getQuaternionFromEuler([0, 0, 1.57]))
    # 创建结束，重新开启渲染
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    num_joints = pybullet.getNumJoints(r_ind)
    # 获得各关节的信息
    joint_infos = []
    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(r_ind, i)
        if joint_info[2] != pybullet.JOINT_FIXED:
            if 'wheel' in str(joint_info[1]):
                print(joint_info)
                joint_infos.append(joint_info)

    maxforce = 10
    velocity = 31.4
    for i in range(len(joint_infos)):
        pybullet.setJointMotorControl2(bodyUniqueId=r_ind,
                                       jointIndex=joint_infos[i][0],
                                       controlMode=pybullet.VELOCITY_CONTROL,
                                       targetVelocity=velocity,
                                       force=maxforce)
    while True:
        pmin, pmax = pybullet.getAABB(r_ind)
        collide_ids = pybullet.getOverlappingObjects(pmin, pmax)
        print(collide_ids)
        for collide_id in collide_ids:
            if collide_id[0] != r_ind:
                print('detect robot collide with object id {}!'.format(r_ind))

        pybullet.stepSimulation()
        time.sleep(1. / 240.)
