#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
// #include "control_msgs/msg/joint_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>


/////// COMMENT OUT FOR NOW ///////
// #include <ros/package.h>


using namespace std::chrono_literals;
using std::placeholders::_1;


/////////////// DEFINITION OF NODE CLASS //////////////

class CartesianController : public rclcpp::Node
{
public:

  const std::string urdf_path_ = "/home/michael/FOR_TESTING/panda.urdf";
  const unsigned int n_joints_ = 7;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  KDL::Tree panda_tree_;
  KDL::Chain panda_chain_;

  //Create solvers
  // KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainIkSolverVel_pinv vel_ik_solver_;
  KDL::ChainIkSolverPos_NR ik_solver_;

  KDL::JntArray joint_pos_start_;
  KDL::Frame task_pos_start_;

  // use moving-average to smoothen the joint values
  long unsigned int num_smooth = 3;
  std::vector<double> q1s {};
  std::vector<double> q2s {};
  std::vector<double> q3s {};
  std::vector<double> q4s {};
  std::vector<double> q5s {};
  std::vector<double> q6s {};
  std::vector<double> q7s {};

    
  CartesianController()
  : Node("cartesian_controller")
  { 
    //Create publisher and subscriber and timer
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&CartesianController::controller_callback, this));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "joint_states", 10, std::bind(&CartesianController::joint_states_callback, this, _1));
    
    //Parse urdf model and generate KDL tree
    if (!kdl_parser::treeFromFile(urdf_path_, panda_tree_)){
      // RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to construct kdl tree");
      rclcpp::shutdown();
    }

    //Generate a kinematic chain from the robot base to its tcp
    panda_tree_.getChain("panda_link0", "panda_grasptarget", panda_chain_);

    //Create solvers
    // KDL::ChainFkSolverPos_recursive fk_solver_(panda_chain_);
    // KDL::ChainIkSolverVel_pinv vel_ik_solver(panda_chain_, 0.0001, 1000);
    // KDL::ChainIkSolverPos_NR ik_solver(panda_chain_, fk_solver_, vel_ik_solver, 1000);

    //Create data structures
    joint_pos_start_ = KDL::JntArray(n_joints_);
    task_pos_start_ = KDL::Frame();

  }

  void controller_callback()
  { 
    auto message = trajectory_msgs::msg::JointTrajectory();
    message.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

    ///////// compute IK
    double joint_results[7] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    message.points = {point};

    RCLCPP_INFO(this->get_logger(), "Publishing joint values: %s", message.points[0]);
    publisher_->publish(message);

  }

  void joint_states_callback(const sensor_msgs::msg::JointState & msg) const
  { 
    auto data = msg.position;
    RCLCPP_INFO(this->get_logger(), "I heard joint values of '%s' from Gazebo!", data);

    // // compute joint pos start
    // for (unsigned int i=0; i<n_joints_; i++) {
    //   joint_pos_start_.data(i, 0) = data.at(i);
    // }

    // compute task pos start
    fk_solver_.JntToCart(joint_pos_start_, task_pos_start_);
    
  }


  ///////////////////////////////////// JOINT CONTROLLER /////////////////////////////////////
  void controller_publisher()
  { 
    if (control == true) {

      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

      ///////// compute IK /////////
      compute_ik(offset, origin, curr_joint_vals, ik_joint_vals);
      if (count < max_count) count++;  // increase count up to the max_count value
      
      ///////// check limits /////////
      if (!within_limits(ik_joint_vals)) {
        std::cout << "--------\nThese violate the joint limits of the Panda arm, shutting down now !!!\n---------" << std::endl;
        rclcpp::shutdown();
      }

      ///////// perform moving-average smoothing /////////
      if (q1s.size() < num_smooth) {

        // don't have enough to perform averaging, push_back more values
        std::cout << "Waiting for the smoothing vector to fill up ... " << std::endl; 
        q1s.push_back(ik_joint_vals.at(0));
        q2s.push_back(ik_joint_vals.at(1));
        q3s.push_back(ik_joint_vals.at(2));
        q4s.push_back(ik_joint_vals.at(3));
        q5s.push_back(ik_joint_vals.at(4));
        q6s.push_back(ik_joint_vals.at(5));
        q7s.push_back(ik_joint_vals.at(6));

      } else {

        // vector is at capacity, start pop/pushing, and averaging, and publishing
        std::cout << "Smoothing vector is full! Start popping / pushing, and averaging " << std::endl; 
        q1s.erase(q1s.begin());
        q2s.erase(q2s.begin());
        q3s.erase(q3s.begin());
        q4s.erase(q4s.begin());
        q5s.erase(q5s.begin());
        q6s.erase(q6s.begin());
        q7s.erase(q7s.begin());
        q1s.push_back(ik_joint_vals.at(0));
        q2s.push_back(ik_joint_vals.at(1));
        q3s.push_back(ik_joint_vals.at(2));
        q4s.push_back(ik_joint_vals.at(3));
        q5s.push_back(ik_joint_vals.at(4));
        q6s.push_back(ik_joint_vals.at(5));
        q7s.push_back(ik_joint_vals.at(6));
        control_joint_vals.at(0) = std::accumulate(q1s.begin(), q1s.end(), 0.0) / q1s.size();
        control_joint_vals.at(1) = std::accumulate(q2s.begin(), q2s.end(), 0.0) / q2s.size();
        control_joint_vals.at(2) = std::accumulate(q3s.begin(), q3s.end(), 0.0) / q3s.size();
        control_joint_vals.at(3) = std::accumulate(q4s.begin(), q4s.end(), 0.0) / q4s.size();
        control_joint_vals.at(4) = std::accumulate(q5s.begin(), q5s.end(), 0.0) / q5s.size();
        control_joint_vals.at(5) = std::accumulate(q6s.begin(), q6s.end(), 0.0) / q6s.size();
        control_joint_vals.at(6) = std::accumulate(q7s.begin(), q7s.end(), 0.0) / q7s.size();

        ///////// do the initial smooth transitioning from current position to Falcon-mapped position /////////
        weight = (double)count / max_count;
        std::cout << "The current count is " << count << std::endl;
        std::cout << "The current weight is " << weight << std::endl;
        for (unsigned int i=0; i<n_joints; i++) message_joint_vals.at(i) = weight * control_joint_vals.at(i) + (1-weight) * curr_joint_vals.at(i);
        
        ///////// prepare the trajectory message, introducing artificial latency (1.5x) /////////
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = message_joint_vals;
        point.time_from_start.nanosec = 100 * 1000000;     //// => {milliseconds} * 1e6

        message.points = {point};

        std::cout << "The joint values [MESSAGE] are ";
        print_joint_vals(message_joint_vals);

        // RCLCPP_INFO(this->get_logger(), "Publishing controller joint values");
        controller_pub_->publish(message);

      }
    }
  }
  
};






void solve_ik() {

  std::string urdf_path = ros::package::getPath("ur5-joint-position-control");
	if(urdf_path.empty()) {
		ROS_ERROR("ur5-joint-position-control package path was not found");
	}
	urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
	ros::init(argc, argv, "tcp_control");

	ros::NodeHandle n;

	ros::Rate loop_rate(loop_rate_val);

	//Create subscribers for all joint states
	ros::Subscriber shoulder_pan_joint_sub = n.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
	ros::Subscriber shoulder_lift_joint_sub = n.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
	ros::Subscriber elbow_joint_sub = n.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
	ros::Subscriber wrist_1_joint_sub = n.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
	ros::Subscriber wrist_2_joint_sub = n.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
	ros::Subscriber wrist_3_joint_sub = n.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);

	//Create publishers to send position commands to all joints
	ros::Publisher joint_com_pub[6]; 
	joint_com_pub[0] = n.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	joint_com_pub[1] = n.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	joint_com_pub[2] = n.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	joint_com_pub[3] = n.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	joint_com_pub[4] = n.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	joint_com_pub[5] = n.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	//Parse urdf model and generate KDL tree
	KDL::Tree ur5_tree;
	if (!kdl_parser::treeFromFile(urdf_path, ur5_tree)){
		ROS_ERROR("Failed to construct kdl tree");
   		return false;
	}

	//Generate a kinematic chain from the robot base to its tcp
	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

	//Create solvers
	KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

	//Make sure we have received proper joint angles already
	for(int i=0; i< 2; i++) {
		ros::spinOnce();
	 	loop_rate.sleep();
	}

	const float t_step = 1/((float)loop_rate_val);
	int count = 0;
	while (ros::ok()) {

		//Compute current tcp position
		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

		ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));		
		ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

		//get user input
		float t_max;
		KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
		get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

		KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

		//Compute inverse kinematics
		KDL::JntArray jnt_pos_goal(Joints);
		ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

		float t = 0.0;
		while(t<t_max) {
			std_msgs::Float64 position[6];
			//Compute next position step for all joints
			for(int i=0; i<Joints; i++) {
				position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
				joint_com_pub[i].publish(position[i]);
			}

			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += t_step;	
		}		
	}	


  return 0;

}








//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   
  rclcpp::init(argc, argv);

  std::shared_ptr<CartesianController> michael = std::make_shared<CartesianController>();

  rclcpp::spin(michael);

  rclcpp::shutdown();
  return 0;
}

