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

    // parameters name list
    std::vector<std::string> param_names = {"part_id", "traj_id", "auto_id"};
    int part_id {0};
    int traj_id {0};
    int auto_id {0};
    
     //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.4559, 0.0, 0.3846};
    const int max_points = 200;
    const double r = 0.1;

    MarkerPublisher()
    : Node("marker_publisher")
    { 
      // parameter stuff
      this->declare_parameter(param_names.at(0), 0);
      this->declare_parameter(param_names.at(1), 0);
      this->declare_parameter(param_names.at(2), 0);
      // this->declare_parameter(param_names.at(0), rclcpp::PARAMETER_INTEGER);
      
      std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
      part_id = std::stoi(params.at(0).value_to_string().c_str());
      traj_id = std::stoi(params.at(1).value_to_string().c_str());
      auto_id = std::stoi(params.at(2).value_to_string().c_str());
      print_params();

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

      // std::cout << "Publishing!\n" << std::endl;
      marker_pub_->publish(line_strip);    // this is a continuous line, good!

    }

    void print_params() {
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
      std::cout << "\n\nThe current parameters [marker_publisher] are as follows:\n" << std::endl;
      std::cout << "Participant ID = " << part_id << "\n" << std::endl;
      std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
      std::cout << "Autonomy ID = " << auto_id << "\n" << std::endl;
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
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