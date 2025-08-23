import rclpy
from rclpy.node import Node
import spot_joints.kinematics as kinematics
import numpy as np
from math import *
from spot_interfaces.msg import Joints
from time import sleep

joint_angles = [0.0] * 12

class Actions(Node):
    def __init__(self):
        super().__init__('joint_actions')
        self.publisher_ = self.create_publisher(Joints, 'joints', 10)
        self.joint_angles = [0.0] * 12
        self.get_logger().info('starting joint_actions node')


    def make_msg(self, angle_list):
        joints = Joints()
        joints.front_right_hip = angle_list[0]
        joints.front_right_knee = angle_list[1]
        joints.front_right_ankle = angle_list[2]
        joints.front_left_hip = angle_list[3]
        joints.front_left_knee = angle_list[4]
        joints.front_left_ankle = angle_list[5]
        joints.back_right_hip = angle_list[6]
        joints.back_right_knee = angle_list[7]
        joints.back_right_ankle = angle_list[8]
        joints.back_left_hip = angle_list[9]
        joints.back_left_knee = angle_list[10]
        joints.back_left_ankle = angle_list[11]
        return joints

    def stand(self, height=50):
        kinematics.Lp = kinematics.Op.copy()
        Fla,Fra,Bla,Bra = kinematics.calcAngles()
        self.joint_angles = Fra + Fla + Bra + Bla
        self.joint_angles = [degrees(angle) for angle in self.joint_angles] 
        self.joint_angles = np.interp(self.joint_angles, (-180,  +180), (0, 180))
        self.publisher_.publish(self.make_msg(self.joint_angles))

    def pose(self, omega = 0, phi = 0, psi = 0):
        Fla,Fra,Bla,Bra = kinematics.calcAngles(omega, phi, psi)
        self.joint_angles = Fra + Fla + Bra + Bla
        self.joint_angles = [degrees(angle) for angle in self.joint_angles] 
        self.joint_angles = np.interp(self.joint_angles, (-180,  +180), (0, 180))
        self.publisher_.publish(self.make_msg(self.joint_angles))

    def calculate_next_pos(self, phase, origin, radius, direction = -1):
            angle = 2 * np.pi * phase
            x = origin[0] + direction*np.cos(angle) * radius
            y = origin[1] + max(-radius*0.9, np.sin(angle) * radius)
            z = origin[2]

            return x, y, z

    def walk(self, num_steps, num_frames, speed = 3):
        phase_origin = [0, 0.5, 0.3, 0.8]
        radius = 30.0
        timestep = 0.008 - (speed * 0.001)
        for step in range(num_steps):
            for frame in range(num_frames+1):
                phase = frame / num_frames
                for index, origin_pos in enumerate(kinematics.Op):
                    x,y,z = self.calculate_next_pos((phase + phase_origin[index]), origin_pos, radius, -1)
                    kinematics.Lp[index] = [x, y, z, 1]
                    Fla,Fra,Bla,Bra = kinematics.calcAngles()
                    #self.joint_angles = Fra + Fla + Bra + Bla
                    self.joint_angles = Fra + Fla + Bra + Bla
                    self.joint_angles = [degrees(angle) for angle in self.joint_angles] 
                    self.joint_angles = np.interp(self.joint_angles, (-180,  +180), (0, 180))
                    self.publisher_.publish(self.make_msg(self.joint_angles))
                    sleep(timestep)
        return
