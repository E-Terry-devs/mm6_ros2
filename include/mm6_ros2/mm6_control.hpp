#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/bind.hpp>


#include <geometry_msgs/msg/twist.hpp>

class MM6Control : public rclcpp::Node
{
public:
    MM6Control();
    ~MM6Control();

private:
    void initialiseMM6();
    void velCmdCallback(const geometry_msgs::msg::Twist::SharedPtr vel);


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    std::string topic_cmd_vel;

};