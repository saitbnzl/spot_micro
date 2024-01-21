import rclpy
from rclpy.node import Node

from spot_interfaces.msg import Commands
from actions import *

import threading

class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

class RemoteSubscriber(Node):
    def __init__(self):
        super().__init__('remote_subscriber')
        self.get_logger().info('starting remote_subscriber node')
        self.actions = Actions()
        self.subscription = self.create_subscription(
            Commands,
            'commands',
            self.remote_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def remote_callback(self, msg):
        self.get_logger().info(f'Command recevied: {msg}')

        if msg.walk:
            self.walkingThread = StoppableThread(target=self.actions.walk, args=(3,30))
            if not self.walkingThread.is_alive():
                self.walkingThread.start()
            return
        elif msg.stop:
            self.walkingThread.stop()
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



