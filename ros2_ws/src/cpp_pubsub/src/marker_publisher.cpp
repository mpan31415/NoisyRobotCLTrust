#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tutorial_interfaces/msg/falconpos.hpp"

using namespace std::chrono_literals;


/////////  functions to show markers in Rviz  /////////
void generate_tcp_marker(visualization_msgs::msg::Marker &tcp_marker);
visualization_msgs::msg::Marker generate_progress(double percentage, double line_width, std::vector<double> center, double height, double width, int id);

void generate_traj_marker(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points,
                          double spiral_r, double spiral_h, bool rotate, int axis, double angle);
void generate_progress_bar(visualization_msgs::msg::Marker &progress_bar, std::vector<double> center, double height, double width);

void get_rotation_matrix(int axis, double angle, std::vector<std::vector<double>> &T);       // axes are: {1-x, 2-y, 3-z}
void matrix_mult_vector(std::vector<std::vector<double>> &mat, std::vector<double> &vec, std::vector<double> &result);


class MarkerPublisher : public rclcpp::Node
{
  public:

    // parameters name list
    std::vector<std::string> param_names = {"part_id", "auto_id", "traj_id"};
    int part_id {0};
    int auto_id {0};
    int traj_id {0};
    
     //////// KEEP CONSISTENT WITH REAL CONTROLLER ////////
    std::vector<double> origin {0.4559, 0.0, 0.3846};
    const int max_points = 200;

    // for the progress bar
    std::vector<double> bar_center {-0.2, 0.0, 1.0};
    double bar_width {0.5};
    double bar_height {0.1};

    const int pub_freq = 20;   // [Hz]
    const int control_freq = 1000;    // [Hz]
    const int max_smoothing_time = 5;   // [seconds]
    const double max_smoothing_count = control_freq * max_smoothing_time;

    double line_width = bar_width / (pub_freq * max_smoothing_time);

    int my_marker_id {5};

    double controller_count {0.0};
    
    std::vector<visualization_msgs::msg::Marker> progress_bar_lines;
  

    MarkerPublisher()
    : Node("marker_publisher")
    { 
      // parameter stuff
      this->declare_parameter(param_names.at(0), 0);
      this->declare_parameter(param_names.at(1), 0);
      this->declare_parameter(param_names.at(2), 0);
      
      std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);
      part_id = std::stoi(params.at(0).value_to_string().c_str());
      auto_id = std::stoi(params.at(1).value_to_string().c_str());
      traj_id = std::stoi(params.at(2).value_to_string().c_str());
      print_params();

      // display the reference trajectory
      switch (traj_id) {
        case 0: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, false, 1, 0); break;
        case 1: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, true, 1, 90); break;
        case 2: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, true, 2, 90); break;
        case 3: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, true, 1, 30); break;
        case 4: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, true, 2, 30); break;
        case 5: generate_traj_marker(traj_marker_, origin, max_points, 0.1, 0.2, true, 1, 70); break;
      }

      // display the empty progress bar
      generate_progress_bar(progress_bar_, bar_center, bar_height, bar_width);

      // create the marker publisher
      marker_timer_ = this->create_wall_timer(20ms, std::bind(&MarkerPublisher::marker_callback, this));  // publish this at 20 Hz
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

      // controller count subscriber
      count_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "controller_count", 10, std::bind(&MarkerPublisher::count_callback, this, std::placeholders::_1));
    }


  private:

    void marker_callback()
    { 
      auto marker_array_msg = visualization_msgs::msg::MarkerArray();

      // add in the certain ones
      marker_array_msg.markers.push_back(progress_bar_);
      marker_array_msg.markers.push_back(traj_marker_);
      generate_tcp_marker(tcp_marker_);
      marker_array_msg.markers.push_back(tcp_marker_);

      // add progress bar if needed
      if (controller_count > 0.0 && controller_count < max_smoothing_count) {

        double percentage = controller_count / max_smoothing_count;
        auto progress_marker = generate_progress(percentage, line_width, bar_center, bar_height, bar_width, my_marker_id);
        my_marker_id++;

        progress_bar_lines.push_back(progress_marker);
        for (size_t i=0; i<progress_bar_lines.size(); i++) marker_array_msg.markers.push_back(progress_bar_lines.at(i));
      }

      marker_pub_->publish(marker_array_msg);

    }

    void count_callback(const std_msgs::msg::Float64 & msg) { controller_count = msg.data; }

    void print_params() {
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
      std::cout << "\n\nThe current parameters [marker_publisher] are as follows:\n" << std::endl;
      std::cout << "Participant ID = " << part_id << "\n" << std::endl;
      std::cout << "Autonomy ID = " << auto_id << "\n" << std::endl;
      std::cout << "Trajectory ID = " << traj_id << "\n" << std::endl;
      for (unsigned int i=0; i<10; i++) std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr marker_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr count_sub_;

    visualization_msgs::msg::Marker traj_marker_;
    visualization_msgs::msg::Marker tcp_marker_;
    visualization_msgs::msg::Marker progress_bar_;
    
};


/////////////////////////////////// FUNCTIONS TO GENERATE TCP MARKER ///////////////////////////////////
void generate_tcp_marker(visualization_msgs::msg::Marker &tcp_marker) 
{
  // fill-in the tcp_marker message
  tcp_marker.header.frame_id = "/panda_hand_tcp";
  tcp_marker.header.stamp = rclcpp::Clock().now();
  tcp_marker.ns = "marker_publisher";
  tcp_marker.action = visualization_msgs::msg::Marker::ADD;
  tcp_marker.id = 0;
  tcp_marker.type = visualization_msgs::msg::Marker::SPHERE;

  // diameters of the sphere in x, y, z directions [cm]
  tcp_marker.scale.x = 0.015;
  tcp_marker.scale.y = 0.015;
  tcp_marker.scale.z = 0.015;

  // sphere is red
  tcp_marker.color.r = 245.0;
  tcp_marker.color.a = 1.0;
  
  // zero offset from the panda tcp link frame
  tcp_marker.pose.position.x = 0.0;
  tcp_marker.pose.position.y = 0.0;
  tcp_marker.pose.position.z = 0.0;
}


/////////////////////////////////// FUNCTIONS TO GENERATE PROGRESS ///////////////////////////////////
visualization_msgs::msg::Marker generate_progress(double percentage, double line_width, std::vector<double> center, double height, double width, int id)
{ 
  auto progress = visualization_msgs::msg::Marker();

  // fill-in the progress message
  progress.header.frame_id = "/panda_link0";
  progress.header.stamp = rclcpp::Clock().now();
  progress.ns = "marker_publisher";
  progress.action = visualization_msgs::msg::Marker::ADD;
  progress.id = id;
  progress.type = visualization_msgs::msg::Marker::LINE_LIST;

  // width of line strip
  progress.scale.x = line_width;

  // line strip is green
  progress.color.g = 1.0;
  progress.color.a = 1.0;
  
  // Create the vertices for the points and lines
  geometry_msgs::msg::Point top;
  geometry_msgs::msg::Point bottom;

  top.x = center.at(0) + 0.0; 
  top.y = center.at(1) - (width/2) + percentage * width;
  top.z = center.at(2) + height/2;

  bottom.x = center.at(0) + 0.0;
  bottom.y = center.at(1) - (width/2) + percentage * width;
  bottom.z = center.at(2) - height/2;

  progress.points.push_back(top);
  progress.points.push_back(bottom);

  return progress;
}


/////////////////////////////////// FUNCTIONS TO GENERATE REFERENCE TRAJECTORY MARKERS ///////////////////////////////////
void generate_traj_marker(visualization_msgs::msg::Marker &traj_marker, std::vector<double> &origin, int max_points,
                          double spiral_r, double spiral_h, bool rotate, int axis, double angle)
{
  // fill-in the traj_marker message
  traj_marker.header.frame_id = "/panda_link0";
  traj_marker.header.stamp = rclcpp::Clock().now();
  traj_marker.ns = "marker_publisher";
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.id = 1;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_marker.scale.x = 0.01;  // make this 1 cm

  // Line strip is blue
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 1.0;

  // get rotation matrix
  std::vector<std::vector<double>> trans_matrix {{1,0,0},{0,1,0},{0,0,1}};
  if (rotate) get_rotation_matrix(axis, angle, trans_matrix);

  std::vector<double> pre_point {0,0,0};
  std::vector<double> post_point {0,0,0};

  // Create the vertices for the points and lines
  for (int count=0; count<=max_points; count++) {

    double t = (double) count / max_points * 2 * M_PI;   // for circle, parametrized in the range [0, 2pi]

    double x = spiral_r * sin(t*2);
    double y = spiral_r * cos(t*2);
    double z = -spiral_h/2 + t/(2*M_PI) * spiral_h;

    pre_point = {x, y, z};
    matrix_mult_vector(trans_matrix, pre_point, post_point);

    geometry_msgs::msg::Point p;
    p.x = post_point.at(0) + origin.at(0);
    p.y = post_point.at(1) + origin.at(1);
    p.z = post_point.at(2) + origin.at(2);

    traj_marker.points.push_back(p);
  }
}


/////////////////////////////////// FUNCTIONS TO GENERATE TCP MARKER ///////////////////////////////////
void generate_progress_bar(visualization_msgs::msg::Marker &progress_bar, std::vector<double> center, double height, double width) 
{
  // fill-in the progress_bar message
  progress_bar.header.frame_id = "/panda_link0";
  progress_bar.header.stamp = rclcpp::Clock().now();
  progress_bar.ns = "marker_publisher";
  progress_bar.action = visualization_msgs::msg::Marker::ADD;
  progress_bar.id = 2;
  progress_bar.type = visualization_msgs::msg::Marker::LINE_LIST;

  // width of line 
  progress_bar.scale.x = 0.02;

  // Points are green
  progress_bar.color.g = 1.0;
  progress_bar.color.a = 1.0;

  // define the positions of the four corners of the progress bar
  geometry_msgs::msg::Point tl;
  geometry_msgs::msg::Point tr;
  geometry_msgs::msg::Point br;
  geometry_msgs::msg::Point bl;

  tl.x = center.at(0) + 0.0; tl.y = center.at(1) - width/2; tl.z = center.at(2) + height/2;
  tr.x = center.at(0) + 0.0; tr.y = center.at(1) + width/2; tr.z = center.at(2) + height/2;
  br.x = center.at(0) + 0.0; br.y = center.at(1) + width/2; br.z = center.at(2) - height/2;
  bl.x = center.at(0) + 0.0; bl.y = center.at(1) - width/2; bl.z = center.at(2) - height/2;

  // add them sequentially, forming line segments between each consecutive pair
  progress_bar.points.push_back(tl);
  progress_bar.points.push_back(tr);
  progress_bar.points.push_back(tr);
  progress_bar.points.push_back(br);
  progress_bar.points.push_back(br);
  progress_bar.points.push_back(bl);
  progress_bar.points.push_back(bl);
  progress_bar.points.push_back(tl);
}



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



/////////////////////////// THE MAIN FUNCTION ///////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}