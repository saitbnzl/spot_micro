import rclpy
from rclpy.node import Node

from spot_interfaces.msg import Commands
from actions import *

import threading
from time import sleep

class RemoteSubscriber(Node):
    def __init__(self):
        super().__init__('remote_subscriber')
        self.get_logger().info('starting remote_subscriber node')
        self.actions = Actions()
        self.walkingThread = None
        self.subscription = self.create_subscription(
            Commands,
            'commands',
            self.remote_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def remote_callback(self, msg):
        self.get_logger().info(f'Command recevied: {msg}')

        if msg.walk:
            if self.walkingThread is not None:
                self.walkingThread.join()
            self.walkingThread = threading.Thread(target=self.actions.walk, args=(1,30))
            self.walkingThread.start()
            return
        elif msg.stop:
            self.walkingThread.join()
            self.actions.stand()
            return

def main(args=None):
    rclpy.init(args=args)

    remote_subscriber = RemoteSubscriber()

    rclpy.spin(remote_subscriber)

    remote_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



