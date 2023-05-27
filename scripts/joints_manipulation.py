#!/usr/bin/python
import os
import time
# import argparse

import rospy
import tf2_ros
import csv

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
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)


        self.joint_states = JointState()
        self.joint_states.name=['joint1','joint2','joint3']
        self.theta1=0
        self.theta2=0
        self.theta3=0
        # self.d_theta=0.01
        self.d_theta=0
        self.dn=-1


        array = np.loadtxt(os.path.join('src/dog_simulation/matlab/test_csv'), delimiter=',')
        print(len(array[1]))
        
        self.theta1=[0.0]*100
        print(self.theta1)
        self.theta2=array[1,:]
        print(self.theta2)
        self.theta3=array[2,:]
        print(self.theta3)

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
        # print('updated')

    def run(self):
        loop = rospy.Rate(10)
        index=0
        while not rospy.is_shutdown():

            if index>=len(self.theta1)-1 or index<=0:
                self.dn=-self.dn

            index=index+self.dn

            self.update_position([self.theta1[index],self.theta2[index],self.theta3[index]])
            self._joint_pub.publish(self.joint_states)

            print(index)
            

            loop.sleep()

    def look_up(self):
        try:
            tf_trans = self._tf_buffer.lookup_transform(
            "odom", "base_link", rospy.Time(0))
            return tf_trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn(e)
            return None


if __name__ == "__main__":

    pub=JointPubisher()
    pub.run()

