#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

import os

import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Falconpos
from std_msgs.msg import Bool


FALCON_MAX_OFFSET = 0.07   # this is 7 [cm]

ORIGIN = [0.4559, 0.0, 0.3846]   # this is in [meters]
MAPPING_RATIO = 2.0

NUM_POINTS = 200

TRAJ_PLOTS_DIR = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/traj_plots/"

FILE_NAME = "control00"



class TrajectoryRecorder(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('traj_recorder')

        # parameter stuff
        self.param_names = ['part_id', 'traj_id', 'auto_id']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], 0),
                (self.param_names[1], 0),
                (self.param_names[2], 0)
                # (self.param_names[2], rclpy.Parameter.Type.INTEGER)
            ]
        )
        (part_param, traj_param, auto_param) = self.get_parameters(self.param_names)
        self.part_id = part_param.value
        self.traj_id = traj_param.value
        self.auto_id = auto_param.value

        self.print_params()

        # tcp position subscriber
        self.tcp_pos_sub = self.create_subscription(Falconpos, 'tcp_position', self.tcp_pos_callback, 10)
        self.tcp_pos_sub  # prevent unused variable warning

        # record flag subscriber
        self.record_flag_sub = self.create_subscription(Bool, 'record', self.record_flag_callback, 10)
        self.record_flag_sub  # prevent unused variable warning

        self.record = False
        self.plot = False
        
        self.xs = []
        self.ys = []
        self.zs = []

        self.lower_bounds = [(ORIGIN[i] - MAPPING_RATIO * FALCON_MAX_OFFSET) for i in range(3)]
        self.upper_bounds = [(ORIGIN[i] + MAPPING_RATIO * FALCON_MAX_OFFSET) for i in range(3)]


    ##############################################################################
    def tcp_pos_callback(self, msg):
        
        if self.record:
            self.get_logger().info('Recording the tcp position: x = %.3f, y = %.3f, z = %.3f ' % (msg.x, msg.y, msg.z))
            self.xs.append(msg.x)
            self.ys.append(msg.y)
            self.zs.append(msg.z)

        if not self.record and len(self.xs) > 0 and not self.plot:
            self.plot = True
            self.plot_trajectory()


    ##############################################################################
    def record_flag_callback(self, msg):

        self.record = msg.data


    ##############################################################################
    def plot_trajectory(self):

        self.ax = plt.figure().add_subplot(projection='3d')
        self.ax.set_xlim(self.lower_bounds[0], self.upper_bounds[0])
        self.ax.set_ylim(self.lower_bounds[1], self.upper_bounds[1])
        self.ax.set_zlim(self.lower_bounds[2], self.upper_bounds[2])

        # Do plotting
        self.ax.plot(self.xs, self.ys, self.zs, label='recorded trajectory')
        self.plot_circle_reference()
        self.ax.legend()

        # saving the figure before I show it (is required)
        plt.savefig(TRAJ_PLOTS_DIR + FILE_NAME + ".png")

        plt.show()


    ##############################################################################
    def plot_circle_reference(self):

        # Spherical coordinates
        r = 0.1  # [meters]
        t = np.linspace(0, 2 * np.pi, NUM_POINTS)

        x = np.zeros(NUM_POINTS)
        y = r * np.sin(t)
        z = r * np.cos(t)

        # adjust for the origin
        for i in range(NUM_POINTS):
            x[i] += ORIGIN[0]
            y[i] += ORIGIN[1]
            z[i] += ORIGIN[2]

        # Do plotting
        self.ax.plot(x, y, z, label='reference trajectory')

    
    ##############################################################################
    def print_params(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters [traj_recorder] are as follows:\n")
        print("The participant_id = %d\n\n" % self.part_id)
        print("The trajectory_id = %d\n\n" % self.traj_id)
        print("The autonomy_id = %d\n\n" % self.auto_id)

        print("=" * 100)
        print("\n" * 10)
        



##############################################################################
def main(args=None):

    rclpy.init(args=args)

    michael = TrajectoryRecorder()

    rclpy.spin(michael)

    michael.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()