#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import sqrt

from tutorial_interfaces.msg import Falconpos
from tutorial_interfaces.msg import PosInfo
from std_msgs.msg import Bool

from cpp_pubsub.data_logger import DataLogger
from cpp_pubsub.traj_utils import get_reference_points

from datetime import datetime
from time import time


ORIGIN = [0.4559, 0.0, 0.3846]   # this is in [meters]

ALL_CSV_DIR = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/"

LOG_DATA = True

TRAJ_DICT_LIST = [
    {'r': 0.1, 'h': 0.2, 'axis': "x", 'angle': 0},
    {'r': 0.1, 'h': 0.2, 'axis': "x", 'angle': 90},
    {'r': 0.1, 'h': 0.2, 'axis': "y", 'angle': 90},
    {'r': 0.1, 'h': 0.2, 'axis': "x", 'angle': 30},
    {'r': 0.1, 'h': 0.2, 'axis': "y", 'angle': 30},
    {'r': 0.1, 'h': 0.2, 'axis': "x", 'angle': 70}
]



class TrajRecorder(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('traj_recorder')

        # parameter stuff
        self.param_names = ['free_drive', 'mapping_ratio', 'part_id', 'auto_id', 'traj_id']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], 0),
                (self.param_names[1], 3.0),
                (self.param_names[2], 0),
                (self.param_names[3], 0),
                (self.param_names[4], 0)
            ]
        )
        (free_drive_param, mapping_ratio_param, part_param, auto_param, traj_param) = self.get_parameters(self.param_names)
        self.free_drive = free_drive_param.value
        self.mapping_ratio = mapping_ratio_param.value
        self.part_id = part_param.value
        self.auto_id = auto_param.value
        self.traj_id = traj_param.value

        self.print_params()

        # get the reference trajectory points
        self.traj_params = TRAJ_DICT_LIST[self.traj_id]
        self.refx, self.refy, self.refz = get_reference_points(200, self.traj_params['r'], self.traj_params['h'], self.traj_params['axis'], 
                                                               self.traj_params['angle'], ORIGIN)

        # tcp position subscriber
        self.tcp_pos_sub = self.create_subscription(PosInfo, 'tcp_position', self.tcp_pos_callback, 10)
        self.tcp_pos_sub  # prevent unused variable warning

        # record flag subscriber
        self.record_flag_sub = self.create_subscription(Bool, 'record', self.record_flag_callback, 10)
        self.record_flag_sub  # prevent unused variable warning

        # flags to control data recording & plotting, and writing to csv file
        self.write_data = LOG_DATA
        self.record = False
        self.data_written = False

        # do not log data if in free-drive mode
        if self.free_drive:
            self.write_data = False
        
        # data points of the trajectory {human, robot, total}
        self.hxs = []
        self.hys = []
        self.hzs = []

        self.rxs = []
        self.rys = []
        self.rzs = []

        self.txs = []
        self.tys = []
        self.tzs = []

        self.times = []
        self.datetimes = []

        # file name of the csv sheet
        self.csv_dir = ALL_CSV_DIR + "part" + str(self.part_id) + "/"


    ##############################################################################
    def tcp_pos_callback(self, msg):
        
        if self.record:
            # self.get_logger().info('Recording the tcp position: x = %.3f, y = %.3f, z = %.3f ' % (msg.x, msg.y, msg.z))
            self.hxs.append(msg.human_position[0])
            self.hys.append(msg.human_position[1])
            self.hzs.append(msg.human_position[2])

            self.rxs.append(msg.robot_position[0])
            self.rys.append(msg.robot_position[1])
            self.rzs.append(msg.robot_position[2])

            self.txs.append(msg.tcp_position[0])
            self.tys.append(msg.tcp_position[1])
            self.tzs.append(msg.tcp_position[2])

            self.times.append(time())
            self.datetimes.append(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

            # print("Appending new data, currently at size %d" % len(self.hxs))

            if len(self.hxs) == 200:
                # print("\n\nWe have finished recording!, writing to csv now!\n\n")
                # we have finished recording points, write to csv file now
                if self.write_data:
                    self.write_to_csv()
                    self.data_written = True
        
        else:
            if len(self.hxs) > 0 and not self.data_written:
                print("\n\nWe have finished recording!, writing to csv now!\n\n")
                # we have finished recording points, write to csv file now
                if self.write_data:
                    self.write_to_csv()
                    self.data_written = True


    ##############################################################################
    def record_flag_callback(self, msg):
        self.record = msg.data


    ##############################################################################
    def write_to_csv(self):

        dl = DataLogger(self.csv_dir, self.part_id, self.auto_id, self.traj_id, self.hxs, self.hys, self.hzs,
                        self.rxs, self.rys, self.rzs, self.txs, self.tys, self.tzs, self.times, self.datetimes)

        dl.write_header()
        dl.log_data()

    
    ##############################################################################
    def print_params(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters [traj_recorder] are as follows:\n")
        print("The free_drive flag = %d\n\n" % self.free_drive)
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