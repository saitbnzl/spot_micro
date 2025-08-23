import rclpy
import subprocess

from rclpy.node import Node

from spot_interfaces.msg import Commands
from spot_joints.actions import *


import threading
from time import sleep

class RemoteSubscriber(Node):
    def __init__(self):
        super().__init__('remote_subscriber')
        self.get_logger().info('starting remote_subscriber node')
        subprocess.Popen(["ros2", "run", "spot_joints", "joint_controller"])
        # Start rosbridge automatically
        self.rosbridge_process = subprocess.Popen(
            ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
        )
        
        self.actions = Actions()
        self.walkingThread = None
        self.subscription = self.create_subscription(
            Commands,
            'commands',
            self.remote_callback,
            10)
        self.subscription  # prevent unused variable warning

    def remote_callback(self, msg):
        self.get_logger().info(f'Command received: {msg}')

        # Stop walking thread if it's running
        if self.walkingThread is not None and self.walkingThread.is_alive():
            self.walkingThread.join()

        if msg.walk:
            self.walkingThread = threading.Thread(target=self.actions.walk, args=(1, 30, msg.speed))
            self.walkingThread.start()

        elif msg.stand:
            self.actions.stand()

        elif msg.lay:
            self.actions.pose(phi=-0.3)  # Lean forward and down for a lay pose

        elif msg.stop:
            self.actions.stand()
            
    def destroy_node(self):
        # Kill rosbridge process on exit
        if hasattr(self, "rosbridge_process"):
            self.rosbridge_process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    remote_subscriber = RemoteSubscriber()

    rclpy.spin(remote_subscriber)

    remote_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



