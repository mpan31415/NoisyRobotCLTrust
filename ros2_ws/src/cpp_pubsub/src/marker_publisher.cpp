#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;


///////// functions to plot reference trajectory
void generate_marker0(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);
void generate_marker1(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points);


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
      auto traj_marker = visualization_msgs::msg::Marker();

      switch (traj_id) {
        case 0: generate_marker0(traj_marker, origin, max_points); break;
        case 1: generate_marker1(traj_marker, origin, max_points); break;
      }

      marker_pub_->publish(traj_marker);    // this is a continuous line, good!
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


/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE TRAJECTORY MARKERS ///////////////////////////////////
void generate_marker0(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points) {

  double r = 0.1;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

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

    traj_marker.points.push_back(p);
  }
}

void generate_marker1(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points) {

  double r = 0.2;

  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.02;  // make this 2 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

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

    traj_marker.points.push_back(p);
  }
}





int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}