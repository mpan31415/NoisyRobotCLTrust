#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



#include <stdio.h>
#include "dhdc.h"


using namespace std::chrono_literals;


/////////////// DEFINITION OF PUBLISHER CLASS //////////////

class EncoderPublisher : public rclcpp::Node
{
public:
  int encCount;
  int enc[DHD_MAX_DOF];

  EncoderPublisher(int a_encCount)
  : Node("encoder_publisher"), count_(0)
  { 
    encCount = a_encCount;
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&EncoderPublisher::timer_callback, this));
  }


private:
  void timer_callback()
  { 

    // apply zero force for convenience
    dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // read all available encoders
    if (dhdGetEnc (enc) < 0) {
      printf ("error: cannot read encoders (%s)\n", dhdErrorGetLastStr ());
    //   done = 1;
      printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
      rclcpp::shutdown();
    }

    // print out encoders according to system type
    for (int i=0; i<encCount; i++) printf ("%06d ", enc[i]);
    printf ("          \r");

    // check for exit condition
    // if (dhdKbHit() && dhdKbGet() == 'q') done = 1;
    if (dhdKbHit() && dhdKbGet() == 'q') {
        printf ("\n\n=============================== THANK YOU FOR FLYING WITH FALCON ===============================\n\n");
        rclcpp::shutdown();
    }


    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++) + " and ==== WE HAVE " + std::to_string(encCount) + " ENCODERS !! ====";
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};





//////////////////// MAIN FUNCTION ///////////////////

int main(int argc, char * argv[])
{   

  ////////////////// INITIALIZE AND SPIN ROS NODE ////////////////
  rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<EncoderPublisher>());



  //////////////////////// FALCON STUFF /////////////////////

//   int i;
//   int done = 0;
//   int enc[DHD_MAX_DOF];
  int encCount;

  // message
  printf ("Force Dimension - Encoder Reading Example %s\n", dhdGetSDKVersionStr());
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

  // identify number of encoders to report based on device type
  switch (dhdGetSystemType ()) {
  case DHD_DEVICE_DELTA3:
  case DHD_DEVICE_OMEGA3:
  case DHD_DEVICE_FALCON:
    encCount = 3;
    break;
  case DHD_DEVICE_OMEGA33:
  case DHD_DEVICE_OMEGA33_LEFT:
    encCount = 6;
    break;
  case DHD_DEVICE_OMEGA331:
  case DHD_DEVICE_OMEGA331_LEFT:
    encCount = 7;
    break;
  case DHD_DEVICE_CONTROLLER:
  case DHD_DEVICE_CONTROLLER_HR:
  default:
    encCount = 8;
    break;
  }

  // display instructions
  printf ("press 'q' to quit\n\n");
  printf ("encoder values:\n");

  // configure device
  dhdEnableExpertMode();



  std::shared_ptr<EncoderPublisher> michael = std::make_shared<EncoderPublisher>(encCount);
//   michael->enc_ = enc;
//   michael->encCount_ = encCount;

// //   rclcpp::spin(std::make_shared<EncoderPublisher>());
  rclcpp::spin(michael);





//   // haptic loop
//   while (!done) {

//     // apply zero force for convenience
//     dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

//     // read all available encoders
//     if (dhdGetEnc (enc) < 0) {
//       printf ("error: cannot read encoders (%s)\n", dhdErrorGetLastStr ());
//       done = 1;
//     }

//     // print out encoders according to system type
//     for (i=0; i<encCount; i++) printf ("%06d ", enc[i]);
//     printf ("          \r");

//     // check for exit condition
//     if (dhdKbHit() && dhdKbGet() == 'q') done = 1;
//   }

//   // close the connection
//   dhdClose ();


//   // happily exit
//   printf ("\ndone.\n");



  //////////////////////////////////////////////////////////////////////////////////


//   rclcpp::shutdown();
  return 0;
}
