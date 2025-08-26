#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class VelocityStamped : public rclcpp::Node {
private:
    float vx;
    float vy;
    float vz;

public:

  VelocityStamped() : Node("velocity_stamped") {
    velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/tecdron_base_controller/cmd_vel_unstamped", 10);
    visual_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/tecdron_visual_controller/cmd_vel_unstamped", 10);

    timer_ = this->create_wall_timer(
        50ms, std::bind(&VelocityStamped::timer_callback, this));

    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&VelocityStamped::velocity_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Velocity Stamped node has been started.");

  }

private:
  void timer_callback() {

    auto message = geometry_msgs::msg::Twist();
    auto message_visual = geometry_msgs::msg::Twist();
    message.linear.x = vx;
    message.linear.y = vy;
    message.angular.z = vz;

    message_visual.linear.x = vx;
    message_visual.linear.y = vy;
    message_visual.angular.z = vz;
    
    velocity_publisher->publish(message);
    visual_velocity_publisher->publish(message_visual);

  }

  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->angular.z;
    
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr visual_velocity_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityStamped>());
  rclcpp::shutdown();
  return 0;
}