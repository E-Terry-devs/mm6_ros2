#include <cstdio>
#include "zbCom/zbCom.hpp"
#include "mm6_ros2/mm6_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<MM6Control> mm6;
    try
    {
        mm6 = std::make_shared<MM6Control>();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    if (!rclcpp::ok())
    {
        std::cerr << "MM failed to start or was stopped by user..." << std::endl;
        return -1;
    }

  rclcpp::spin(mm6);

  rclcpp::shutdown();
  return 0;
}
