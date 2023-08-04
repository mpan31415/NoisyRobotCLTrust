#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;





class MarkerPublisher : public rclcpp::Node
{
  public:

     //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.4559, 0.0, 0.3846};
    const int max_points = 200;
    const double r = 0.1;

    MarkerPublisher()
    : Node("marker_publisher")
    {   
      // create the marker publisher
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
      marker_timer_ = this->create_wall_timer(20ms, std::bind(&MarkerPublisher::marker_callback, this));  // publish this at 50 Hz
    }

  private:
    void marker_callback()
    {
      auto line_strip = visualization_msgs::msg::Marker();

      // fill-in the line_strip message
      line_strip.header.frame_id = "/panda_link0";
      line_strip.header.stamp = rclcpp::Clock().now();
      line_strip.ns = "marker_publisher";
      line_strip.action = visualization_msgs::msg::Marker::ADD;
      line_strip.id = 0;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;

      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      line_strip.scale.x = 0.02;  // make this 2 cm

      // Line strip is blue
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

      // Create the vertices for the points and lines
      for (int count=0; count<=max_points; count++) {

        double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

        double x = 0.0 + origin.at(0);
        double y = r * sin(t) + origin.at(1);
        double z = r * cos(t) + origin.at(2);

        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        line_strip.points.push_back(p);
      }

      std::cout << "Publishing!\n" << std::endl;
      marker_pub_->publish(line_strip);    // this is a continuous line, good!

    }


    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}