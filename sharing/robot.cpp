// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_hardware/robot.hpp>

#include <cassert>
#include <mutex>

#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>


///////////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/srv/set_force_torque_collision_behavior.hpp"
#include <string.h>
#include <malloc.h>
///////////////////////////////////////////////////////


namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  tau_command_.fill(0.);
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
}

void Robot::write(const std::array<double, 7>& efforts) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}

void Robot::stopRobot() {
  if (!stopped_) {
    finish_ = true;
    control_thread_->join();
    finish_ = false;
    stopped_ = true;
  }
}

void Robot::initializeTorqueControl() {
  assert(isStopped());
  stopped_ = false;
  const auto kTorqueControl = [this]() {
    robot_->control(
        [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
          {
            std::lock_guard<std::mutex> lock(read_mutex_);
            current_state_ = state;
          }
          std::lock_guard<std::mutex> lock(write_mutex_);
          franka::Torques out(tau_command_);
          out.motion_finished = finish_;
          return out;
        },
        true, franka::kMaxCutoffFrequency);
  };
  control_thread_ = std::make_unique<std::thread>(kTorqueControl);
}

void Robot::initializeContinuousReading() {
  assert(isStopped());
  stopped_ = false;
  const auto kReading = [this]() {
    robot_->read([this](const franka::RobotState& state) {
      {
        std::lock_guard<std::mutex> lock(read_mutex_);
        current_state_ = state;
      }
      return !finish_;
    });
  };
  control_thread_ = std::make_unique<std::thread>(kReading);
}


//////////////////////////////////////////////////////////////////////////////////////////////

void Robot::setForceTorqueCollisionBehavior()
{
  std::cout << "Setting collision behavior" << std::endl;

  std::array<double, 7> lower_torque_thresholds_nominal {1, 2, 3, 4, 5, 6, 7};
  std::array<double, 7> upper_torque_thresholds_nominal {1, 2, 3, 4, 5, 6, 7};

  std::array<double, 6> lower_force_thresholds_nominal {1, 2, 3, 4, 5, 6};
  std::array<double, 6> upper_force_thresholds_nominal {1, 2, 3, 4, 5, 6};

  robot_->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                             lower_force_thresholds_nominal, upper_force_thresholds_nominal);

  std::cout << "Finished setting the requested collision behavior of the robot!" << std::endl;
}

// void Robot::setForceTorqueCollisionBehavior(
//     const std::shared_ptr<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request> req,
//           std::shared_ptr<franka_msgs::srv::SetForceTorqueCollisionBehavior::Response> res)
// {
//   std::cout << "Received incoming request" << std::endl;

//   std::array<double, 7> lower_torque_thresholds_nominal;
//   std::copy(req->lower_torque_thresholds_nominal.cbegin(),
//             req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
//   std::array<double, 7> upper_torque_thresholds_nominal;
//   std::copy(req->upper_torque_thresholds_nominal.cbegin(),
//             req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
//   std::array<double, 6> lower_force_thresholds_nominal;
//   std::copy(req->lower_force_thresholds_nominal.cbegin(), req->lower_force_thresholds_nominal.cend(),
//             lower_force_thresholds_nominal.begin());
//   std::array<double, 6> upper_force_thresholds_nominal;
//   std::copy(req->upper_force_thresholds_nominal.cbegin(), req->upper_force_thresholds_nominal.cend(),
//             upper_force_thresholds_nominal.begin());

//   robot_->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
//                              lower_force_thresholds_nominal, upper_force_thresholds_nominal);

//   std::cout << "Finished setting the requested collision behavior of the robot!" << std::endl;
//   res->success = true;
// }


// void Robot::enableCollisionSetting() {

//   char** argv = (char**)malloc(strlen("1\0"));

//   rclcpp::init(1, argv);

//   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_ft_cb_server");

//   // rclcpp::Service<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr service =
//   //   node->create_service<franka_msgs::srv::SetForceTorqueCollisionBehavior>("set_ft_cb", &setForceTorqueCollisionBehavior);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set force/torque collision behavior!");

//   rclcpp::spin(node);
// }


//////////////////////////////////////////////////////////////////////////////////////////////


Robot::~Robot() {
  stopRobot();
}

bool Robot::isStopped() const {
  return stopped_;
}
}  // namespace franka_hardware
