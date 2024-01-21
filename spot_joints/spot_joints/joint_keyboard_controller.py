import rclpy
from rclpy.node import Node

from actions import *

class JointKeyboard(Node):

    def __init__(self):
        super().__init__('joint_keyboard_controller')
        self.get_logger().info('starting joint_keyboard_controller node')
        self.actions = Actions()

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

    
    def shell(self, string):
        args = string.split()
        if args[0].lower() == "lay":
            self.get_logger().info('laying down')
            self.stand(-150)
        elif args[0].lower() == "stand":
            self.get_logger().info('standing up')
            try:
                self.stand(float(args[1]))
            except:
                self.stand(50)
        elif args[0].lower() == "w":
            self.actions.walk(100, 50, 5)
            self.get_logger().info('walking')
        else:
            self.get_logger().info('input error')



def main(args=None):
    rclpy.init(args=args)

    joint_keyboard = JointKeyboard()

    while True:
        joint_keyboard.shell(input("> "))

    joint_keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
