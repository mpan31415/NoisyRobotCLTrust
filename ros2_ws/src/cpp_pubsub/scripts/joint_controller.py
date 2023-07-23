import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration


class JointPosPublisher(Node):

    def __init__(self):

        super().__init__('joint_pos_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

        self.published = False


    def timer_callback(self):
        
        if not self.published:

            # time_from_start = Duration()
            # time_from_start.sec = 5

            point1 = JointTrajectoryPoint()
            point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            point1.time_from_start.sec = 5

            point2 = JointTrajectoryPoint()
            point2.positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            point2.time_from_start.sec = 10

            msg = JointTrajectory()
            msg.joint_names = self.joint_names
            msg.points = [point1, point2]
            
            self.get_logger().info('Publishing: "%s"' % msg.points[0].positions)
            self.publisher_.publish(msg)

            self.published = True

            


def main(args=None):

    rclpy.init(args=args)

    joint_pos_pub = JointPosPublisher()

    rclpy.spin(joint_pos_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_pos_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()