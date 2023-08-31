#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "motion_generator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/// The move to start example controller moves the robot into default pose.
class MyController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;


 private:
  std::string arm_id_;
  const int num_joints = 7;

  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;

  Vector7d torque_limits_;
  bool controlling_;


  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub1_;
  rclcpp::GuardCondition::SharedPtr guard_condition1_;

  int wait_time_;
  Vector7d tau_d_calculated_;

  int empty_count_;
  int total_count_;
  bool counting_;
  

  void updateJointStates();
  void withinLimits();

};
}  // namespace franka_example_controllers
