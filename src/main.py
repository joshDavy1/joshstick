#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from iiwa_msgs.msg import JointPosition
from spatialmath import SE3, SO3
from spatialmath.base import q2r
import kuka_iiwa

POS_DELTA = 0.003
ORI_DELTA = 0.01

LEFT_STICK_LR = 5
LEFT_STICK_UD = 4

RIGHT_STICK_LR = 2
RIGHT_STICK_UD = 3

DPAD_UD = 1

ROLL_L = 3
ROLL_R = 2

class Traj:
    def __init__(self):
        rospy.init_node('joystick', anonymous=True)
        rospy.Subscriber('iiwa_1/state/JointPosition', JointPosition, self.iiwa_joint_position_callback)
        self.robot_position_publisher = rospy.Publisher("/iiwa_1/command/JointPosition", JointPosition, queue_size=10)
        rospy.Subscriber('joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(15)
        self.robot = kuka_iiwa.Kuka_iiwa()
        self.robot.q = self.robot.q0

        self.joints = np.zeros(7)
        self.joints_rt = np.zeros(7)
        self.joints_recieved =  False

        self.joy = None
        self.new_joy_recieved = False

        self.delta = SE3()
    
        while not self.joints_recieved:
            rospy.loginfo("Waiting for Joints...")
            self.rate.sleep()
        rospy.loginfo("Joints Recieved")
        self.joints = self.joints_rt
        while not self.new_joy_recieved:
            rospy.loginfo("Waiting for Joystick...")
            self.rate.sleep()
        rospy.loginfo("Joy Connected")

        while not rospy.is_shutdown():
            pose = self.robot.fkine(self.joints)
            pose = pose * self.delta
            sol = self.robot.ikine_LM(pose, q0 = self.joints)

            if sol.success and np.linalg.norm(sol.q - self.joints_rt) < np.deg2rad(50):
                self.position_publish(sol.q)
                self.joints = sol.q
            
            self.rate.sleep()
        
    def iiwa_joint_position_callback(self, iiwa_joint_position_msg):
        self.joints_rt[0] = iiwa_joint_position_msg.position.a1
        self.joints_rt[1] = iiwa_joint_position_msg.position.a2
        self.joints_rt[2] = iiwa_joint_position_msg.position.a3
        self.joints_rt[3] = iiwa_joint_position_msg.position.a4 
        self.joints_rt[4] = iiwa_joint_position_msg.position.a5 
        self.joints_rt[5] = iiwa_joint_position_msg.position.a6 
        self.joints_rt[6] = iiwa_joint_position_msg.position.a7
        self.joints_recieved = True


    def joy_callback(self, msg):
        self.new_joy_recieved  = True
        self.delta = SE3()
        if msg.axes[LEFT_STICK_LR] < -0.2 :
            self.delta.x = -POS_DELTA
        elif msg.axes[LEFT_STICK_LR] > 0.2:
            self.delta.x = POS_DELTA

        if msg.axes[LEFT_STICK_UD] < -0.2 :
            self.delta.y = POS_DELTA
        elif msg.axes[LEFT_STICK_UD] > 0.2:
            self.delta.y = -POS_DELTA

        if msg.axes[DPAD_UD] < -0.2 :
            self.delta.z = POS_DELTA
        elif msg.axes[DPAD_UD] > 0.2:
            self.delta.z = -POS_DELTA

        ori = np.zeros(3)

        if msg.axes[RIGHT_STICK_LR] < -0.2 :
            ori[2] = -ORI_DELTA
        elif msg.axes[RIGHT_STICK_LR] > 0.2:
            ori[2] = ORI_DELTA

        if msg.axes[RIGHT_STICK_UD] < -0.2 :
            ori[1] = ORI_DELTA
        elif msg.axes[RIGHT_STICK_UD] > 0.2:
            ori[1] = -ORI_DELTA

        if msg.buttons[ROLL_L]:
            ori[0] = ORI_DELTA
        if msg.buttons[ROLL_R]:
            ori[0] = -ORI_DELTA
        
        self.delta =  self.delta * SE3.RPY(ori)

    def position_publish(self, q):
        """ Publish positon to real robot"""
        msg = JointPosition()
        msg.position.a1 = q[0]
        msg.position.a2 = q[1]
        msg.position.a3 = q[2]
        msg.position.a4 = q[3]
        msg.position.a5 = q[4]
        msg.position.a6 = q[5]
        msg.position.a7 = q[6]
        self.robot_position_publisher.publish(msg)
        

if __name__ == '__main__':
    try:
        t = Traj()
        t.spin()
    except rospy.ROSInterruptException:
        pass