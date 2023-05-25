#!/usr/bin/python
import os
import time
# import argparse

import rospy
import tf2_ros

import numpy as np

from std_msgs.msg import Float32 

from sensor_msgs.msg import JointState

from datetime import datetime

# os.chdir('')
# os.path.abspath(__file__)
# os.path.join(os.getcwd(), 'data')
# os.path.join(os.path.abspath(__file__), '..', 'data')

# Joint_state
# header: 
#   seq: 4560
#   stamp: 
#     secs: 1684911626
#     nsecs: 745556831
#   frame_id: ''
# name: 
#   - joint_b_l1
#   - joint_l1_l2
#   - joint_l2_l3
#   - joint_b_r1
#   - joint_r1_r2
#   - joint_r2_r3
# position: [-0.0005139999999999034, -0.00029600000000001847, 0.0, -0.0005139999999999034, -0.3276679999999996, -0.06490499999999999]
# velocity: []
# effort: []



class JointPubisher():
    def __init__(self):
        rospy.init_node("joint_publisher")

        self._joint_pub = rospy.Publisher('/joint_states',JointState,queue_size=10)
        self.theta1=np.linspace(0,np.pi/2,100)
        
        self.joint_states = JointState()
        self.joint_states.name=['joint1','joint2','joint3']
        self.theta1=0
        self.theta2=0
        self.theta3=0
        self.d_theta=0.01


        # self._tf_buffer = tf2_ros.Buffer()
        # self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # self._tf_br = tf2_ros.TransformBroadcaster()
        # self._est_pose_pub = rospy.Publisher("/est/pose", PoseArray, queue_size= 10)
        # self._compass_pose_pub = rospy.Publisher("/compasss/pose", PoseArray, queue_size= 10)
        # self._enu_publisher = rospy.Publisher("/enu_angle", Pose, queue_size=1)
        # self._compass_statue = True
        # self._compass_statue_counter = 0
        # rospy.Subscriber("/imu/data", Imu, self._imu_callback)
        # rospy.Subscriber("/mag_feedback", Float32, self._mag_callback)

        time.sleep(1)

    def update_position(self,position):
        self.joint_states.header.stamp=rospy.Time.now()

        self.joint_states.position=position
        print('updated')

    def run(self):
        loop = rospy.Rate(10)
        while not rospy.is_shutdown():

            if abs(self.theta1)>np.pi/6:
                self.d_theta=-self.d_theta

            self.theta1+=self.d_theta
            self.theta2+=self.d_theta
            self.theta3+=self.d_theta
            

            self.update_position([self.theta1,self.theta2,self.theta3])
            self._joint_pub.publish(self.joint_states)
            loop.sleep()


if __name__ == "__main__":

    pub=JointPubisher()
    pub.run()

