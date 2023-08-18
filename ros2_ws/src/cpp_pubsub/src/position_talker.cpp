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


//////////////////////// UTILS FUNCTIONS //////////////////////// 
void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T);       // axes are: {1-x, 2-y, 3-z}
void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result);


/////////////// DEFINITION OF PUBLISHER CLASS //////////////

class PositionTalker : public rclcpp::Node
{
public:

  // parameters name list
  std::vector<std::string> param_names = {"mapping_ratio", "part_id", "auto_id", "traj_id"};
  double mapping_ratio {2.0};
  int part_id {0};
  int auto_id {0};
  int traj_id {0};

  // other arrays
  double p[3] {0.0, 0.0, 0.0};
  double v[3] {0.0, 0.0, 0.0};
  double f[3] {0.0, 0.0, 0.0};
  double K[3] {200.0, 70.0, 70.0};     //////////////////// -> this is the initial gain vector K, will be changed after a few seconds!
  double C[3] {5.0, 5.0, 5.0};      //////////// -> damping vector C, having values higher than 5 will likely cause vibrations
  int choice;

  ///////// -> this is the centering / starting Falcon pos, but is NOT THE ORIGIN => ORIGIN IS ALWAYS (0, 0, 0)
  ///////// -> max bounds are around +-0.05m (5cm)
  ///////// -> this depends on the auto_id parameter
  std::vector<double> centering {0.00, 0.00, 0.00};   ///////// -> note: this is in [meters]
  // guide:
  // {x, y, z} = {1, 2, 3} DOFS = {in/out, left/right, up/down}
  // positive axes directions are {out, right, up}

  // for spiral dimensions and rotation
  std::vector<std::vector<double>> trans_matrix {{1,0,0}, {0,1,0}, {0,0,1}};   // initialized as the identity matrix
  double spiral_r = 0.0;
  double spiral_h = 0.0;
  std::vector<double> pre_point {0.00, 0.00, 0.00};
  std::vector<double> post_point {0.00, 0.00, 0.00};


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  PositionTalker(int a_choice)
  : Node("position_talker")
  { 
    choice = a_choice;

    // parameter stuff
    this->declare_parameter(param_names.at(0), 2.0);
    this->declare_parameter(param_names.at(1), 0);
    this->declare_parameter(param_names.at(2), 0);
    this->declare_parameter(param_names.at(3), 0);
    
    std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
    mapping_ratio = std::stod(params.at(0).value_to_string().c_str());
    part_id = std::stoi(params.at(1).value_to_string().c_str());
    auto_id = std::stoi(params.at(2).value_to_string().c_str());
    traj_id = std::stoi(params.at(3).value_to_string().c_str());
    print_params();

    // get the spiral dimensions & the corresponding transformation matrix
    switch (traj_id) {
      case 0: spiral_r = 0.1; spiral_h = 0.2; break;
      case 1: get_rotation_matrix(1, 90, trans_matrix); spiral_r = 0.1; spiral_h = 0.2; break;
      case 2: get_rotation_matrix(2, 90, trans_matrix); spiral_r = 0.1; spiral_h = 0.2; break;
      case 3: get_rotation_matrix(1, 30, trans_matrix); spiral_r = 0.1; spiral_h = 0.2; break;
      case 4: get_rotation_matrix(2, 30, trans_matrix); spiral_r = 0.1; spiral_h = 0.2; break;
      case 5: get_rotation_matrix(1, 70, trans_matrix); spiral_r = 0.1; spiral_h = 0.2; break;
    }

    // get the first trajectory point
    pre_point = {0.0, spiral_r, -spiral_h/2};
    matrix_mult_vector(trans_matrix, pre_point, post_point);

    // update centering position using "post_point" computed above
    for (size_t i=0; i<3; i++) centering.at(i) = post_point.at(i) / mapping_ratio;

    // publisher
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Falconpos>("falcon_position", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&PositionTalker::timer_callback, this)); ///////// publishing at 1000 Hz /////////
  }


private:

  void timer_callback()
  { 
    ///////////////////////// FALCON STUFF /////////////////////////
    dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
    dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));

    if (count < count_thres2) {
      // gradually perform centering {in increasing levels of K = 1000 -> K = 2000, after 1 -> 2 seconds}
      for (int i=0; i<3; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i];
    } else {
      switch (choice) {
        case 0: for (int i=0; i<3; i++) f[i] = 0; break;
        case 1: for (int i=0; i<1; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
        case 2: for (int i=0; i<2; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
        case 3: for (int i=0; i<3; i++) f[i] = - K[i] * (p[i] - centering[i]) - C[i] * v[i]; break;
      }
    }

    if (dhdSetForceAndTorqueAndGripperForce (f[0], f[1], f[2], 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
      printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
      printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
      rclcpp::shutdown();
    }

    // generate and publish the message
    auto message = tutorial_interfaces::msg::Falconpos();
    message.x = p[0] * 100;
    message.y = p[1] * 100;
    message.z = p[2] * 100;
    // RCLCPP_INFO(this->get_logger(), "Publishing position: px = %.3f, py = %.3f, pz = %.3f  [in cm]", message.x, message.y, message.z);
    publisher_->publish(message);



    if (dhdKbHit() && dhdKbGet() == 'q') {
        printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
        rclcpp::shutdown();
    }

    // only run if the button is pressed (or) the reset condition is active
    if (dhdGetButton (0) == DHD_ON || reset == true) {
      if (!reset) {
        reset = true;
        printf ("\n\n=============================== RE-CENTERING IN 1 SECOND ===============================\n\n");
      }
      reset_count++;
      if (reset_count == count_thres1) {
        // this is 1 second after the button has been pressed
        // need to reset {count, reset_count, reset, K, C}
        printf ("\n\n=============================== CENTERING NOW !!! ===============================\n\n");
        count = 0;
        reset_count = 0;
        reset = false;

        // restore the K and C gain vectors to initial values
        K[0] = 200.0; K[1] = 70.0; K[2] = 70.0;
        C[0] = 5.0;   C[1] = 5.0;   C[2] = 5.0;
      }
    }

    count++;
    if (count > count_thres1) {
      K[0] = 700.0; K[1] = 400.0; K[2] = 400.0;
    }
    if (count > count_thres2) {
      K[0] = 2000.0; K[1] = 1500.0; K[2] = 1500.0;
    }

  }

  void print_params() {
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
    std::cout << "\n\nThe current parameters [position_publisher] are as follows:\n" << std::endl;
    std::cout << "Mapping ratio = " << mapping_ratio << "\n" << std::endl;
    std::cout << "Participant ID = " << part_id << "\n" << std::endl;
    std::cout << "Autonomy ID = " << auto_id << "\n" << std::endl;
    std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
    for (unsigned int i=0; i<10; i++) std::cout << "\n";
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Falconpos>::SharedPtr publisher_;

  const int count_thres1 = 1000;   // 1 second
  const int count_thres2 = 1500;   // 1.5 seconds
  const int count_thres3 = 2000;   // 2 seconds

  int count {0};
  int reset_count {0};

  bool reset = false;

};



/////////////////////////// util functions ///////////////////////////

void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T)
{    
  double th = angle / 180 * M_PI;
  switch (axis) {
      case 1: T.at(0) = {1,0,0}; T.at(1) = {0,cos(th),-sin(th)}; T.at(2) = {0,sin(th),cos(th)}; break;
      case 2: T.at(0) = {cos(th),0,sin(th)}; T.at(1) = {0,1,0}; T.at(2) = {-sin(th),0,cos(th)}; break;
      case 3: T.at(0) = {cos(th),-sin(th),0}; T.at(1) = {sin(th),cos(th),0}; T.at(2) = {0,0,1}; break;
  }
}

void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result) 
{   
  for (size_t i=0; i<mat.size(); i++) {
      auto row = mat.at(i);
      double sum {0};
      for (size_t j=0; j<row.size(); j++) {
          sum += row.at(j) * vec.at(j);
      }
      result.at(i) = sum;
  }
}




//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   

  rclcpp::init(argc, argv);

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

  // guide:
  // {x, y, z} = {1, 2, 3} DOFS = {in/out, left/right, up/down}
  // positive axes directions are {out, right, up}

  int choice = 0;

  ///////////////// CHOOSE YOUR MODE! /////////////////



  std::shared_ptr<PositionTalker> michael = std::make_shared<PositionTalker>(choice);

  rclcpp::spin(michael);

//   rclcpp::shutdown();
  return 0;
}
