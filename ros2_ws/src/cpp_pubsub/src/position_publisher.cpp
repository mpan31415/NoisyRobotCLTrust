#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"  

#include <stdio.h>
#include "dhdc.h"


using namespace std::chrono_literals;


/////////////// DEFINITION OF PUBLISHER CLASS //////////////

class PositionPublisher : public rclcpp::Node
{
public:

  double p[3] {0.0, 0.0, 0.0};
  double v[3] {0.0, 0.0, 0.0};
  double f[3] {0.0, 0.0, 0.0};
  double K = 200.0;
  double C = 5.0;      //////////// -> having this higher will cause vibrations
  int choice;

  PositionPublisher(int a_choice)
  : Node("position_publisher")
  { 
    choice = a_choice;
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Falconpos>("falcon_position", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&PositionPublisher::timer_callback, this));
  }


private:
  void timer_callback()
  { 
    ///////////////////////// FALCON STUFF /////////////////////////
    dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
    dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));

    switch (choice) {
        case 0: for (int i=0; i<3; i++) f[i] = 0; break;
        case 1: for (int i=0; i<1; i++) f[i] = - K * p[i] - C * v[i]; break;
        case 2: for (int i=0; i<2; i++) f[i] = - K * p[i] - C * v[i]; break;
        case 3: for (int i=0; i<3; i++) f[i] = - K * p[i] - C * v[i]; break;
    }

    if (dhdSetForceAndTorqueAndGripperForce (f[0], f[1], f[2], 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
      printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
      rclcpp::shutdown();
    }

    auto message = tutorial_interfaces::msg::Falconpos();
    message.x = p[0] * 100;
    message.y = p[1] * 100;
    message.z = p[2] * 100;
    RCLCPP_INFO(this->get_logger(), "Publishing position: px = %.3f, py = %.3f, pz = %.3f  [in cm]", message.x, message.y, message.z);
    publisher_->publish(message);

    if (dhdKbHit() && dhdKbGet() == 'q') {
        printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
        rclcpp::shutdown();
    }

    count++;
    if (count > count_thres1) K = 1000.0;
    if (count > count_thres2) K = 2000.0;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Falconpos>::SharedPtr publisher_;
  const int count_thres1 = 1000;   // -> corresponds to 1 second wait time
  const int count_thres2 = 2000;   // -> corresponds to 2 second wait time
  int count {0};
};





//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   

  ////////////////// INITIALIZE AND SPIN ROS NODE ////////////////
  rclcpp::init(argc, argv);



  //////////////////////// FALCON STUFF /////////////////////

  // message
  printf ("Force Dimension - Position Example (By Michael Pan) %s\n", dhdGetSDKVersionStr());
  printf ("Copyright (C) 2001-2022 Force Dimension\n");
  printf ("All Rights Reserved.\n\n");

  // open the first available device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());

  // display instructions
  printf ("      'q' to perform landing :) \n\n");

  // enable force and button emulation, disable velocity threshold
  dhdEnableExpertMode ();
  dhdSetVelocityThreshold (0);
  dhdEnableForce (DHD_ON);
  dhdEmulateButton (DHD_ON);

  

  ///////////////// CHOOSE YOUR MODE! /////////////////
  int choice = 1;


  std::shared_ptr<PositionPublisher> michael = std::make_shared<PositionPublisher>(choice);

  rclcpp::spin(michael);







  //////////////////////////////////////////////////////////////////////////////////


//   rclcpp::shutdown();
  return 0;
}
