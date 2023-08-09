#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"


class StaticFramePublisher : public rclcpp::Node
{
public:

  std::string panda_link_name = "panda_link0";
  std::string camera_link_name = "camera_base";
  
  double dx = 0.7;
  double dy = 0.0;
  double dz = 0.0;

  double rx = 0.0;
  double ry = 0.0;
  double rz = 3.14;

  explicit StaticFramePublisher()
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

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

};



int main(int argc, char * argv[])
{ 
  // initialize node and spin it
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}