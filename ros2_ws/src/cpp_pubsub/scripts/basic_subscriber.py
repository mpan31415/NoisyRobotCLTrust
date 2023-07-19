#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from cpp_pubsub.module_to_import import *

from tutorial_interfaces.msg import Falconpos


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Falconpos,
            'falcon_position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Hearing the Falcon position: px = %.3f, py = %.3f, pz = %.3f  [in cm] ----->   and sqrt(5) = %.3f' % (msg.x, msg.y, msg.z, sqrt(5)))



def main(args=None):

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    print("Created the minimal subscriber node! \n\nSpinning it now ... ")

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()