#include "rclcpp/rclcpp.hpp"
#include "ardupilot_gazebo_sitl_viewer/SitlViewer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SitlViewer>());
  rclcpp::shutdown();
  return 0;
}

