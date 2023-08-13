#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"


class ConstBr : public rclcpp::Node
{
public:

  std::string panda_link_name = "panda_link0";
  std::string camera_link_name = "camera_base";

  // define transformation values
  // note: TF does translation before rotation
  double dx = 1.22;
  double dy = 0.6;
  double dz = 1.15;
  double rx = deg2rad(183);
  double ry = deg2rad(30);
  double rz = deg2rad(212);

  ConstBr()
  : Node("const_br")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms();
  }


private:

  void make_transforms()
  { 
    // generate message
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = panda_link_name;
    t.child_frame_id = camera_link_name;

    // translations
    t.transform.translation.x = dx;
    t.transform.translation.y = dy;
    t.transform.translation.z = dz;

    // rotations
    tf2::Quaternion q;
    q.setRPY(rx, ry, rz);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  double deg2rad(double deg) {
    return (deg / 180) * M_PI;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

};



int main(int argc, char * argv[])
{ 
  // initialize node and spin it
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConstBr>());
  rclcpp::shutdown();
  return 0;
}