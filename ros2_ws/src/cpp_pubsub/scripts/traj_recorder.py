#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Falconpos
from std_msgs.msg import Bool

from cpp_pubsub.data_logger import DataLogger


ORIGIN = [0.4559, 0.0, 0.3846]   # this is in [meters]

ALL_CSV_DIR = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/"



class TrajRecorder(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('traj_recorder')

        # parameter stuff
        self.param_names = ['mapping_ratio', 'part_id', 'auto_id', 'traj_id']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], 2.0),
                (self.param_names[1], 0),
                (self.param_names[2], 0),
                (self.param_names[3], 0)
            ]
        )
        (mapping_ratio_param, part_param, auto_param, traj_param) = self.get_parameters(self.param_names)
        self.mapping_ratio = mapping_ratio_param.value
        self.part_id = part_param.value
        self.auto_id = auto_param.value
        self.traj_id = traj_param.value

        self.print_params()

        # tcp position subscriber
        self.tcp_pos_sub = self.create_subscription(Falconpos, 'tcp_position', self.tcp_pos_callback, 10)
        self.tcp_pos_sub  # prevent unused variable warning

        # record flag subscriber
        self.record_flag_sub = self.create_subscription(Bool, 'record', self.record_flag_callback, 10)
        self.record_flag_sub  # prevent unused variable warning

        # flags to control data recording & plotting, and writing to csv file
        self.record = False
        self.write_data = True
        self.data_written = False
        
        # data points of the trajectory
        self.xs = []
        self.ys = []
        self.zs = []

        # file name of the csv sheet
        self.csv_dir = ALL_CSV_DIR + "part" + str(self.part_id) + "/"


    ##############################################################################
    def tcp_pos_callback(self, msg):
        
        if self.record:
            self.get_logger().info('Recording the tcp position: x = %.3f, y = %.3f, z = %.3f ' % (msg.x, msg.y, msg.z))
            self.xs.append(msg.x)
            self.ys.append(msg.y)
            self.zs.append(msg.z)

        if not self.record and len(self.xs) > 0 and not self.data_written:
            # we have finished recording points, write to csv file now
            if self.write_data:
                self.write_to_csv()
                self.data_written = True


    ##############################################################################
    def record_flag_callback(self, msg):
        self.record = msg.data


    ##############################################################################
    def write_to_csv(self):

        dl = DataLogger(self.csv_dir, self.part_id, self.auto_id, self.traj_id, self.xs, self.ys, self.zs)

        dl.write_header()
        dl.log_data()

    
    ##############################################################################
    def print_params(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters [traj_recorder] are as follows:\n")
        print("The mapping_ratio = %d\n\n" % self.mapping_ratio)
        print("The participant_id = %d\n\n" % self.part_id)
        print("The autonomy_id = %d\n\n" % self.auto_id)
        print("The trajectory_id = %d\n\n" % self.traj_id)

        print("=" * 100)
        print("\n" * 10)
        



##############################################################################
def main(args=None):

    rclpy.init(args=args)

    michael = TrajRecorder()

    rclpy.spin(michael)

    michael.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()