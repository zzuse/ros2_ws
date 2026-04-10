#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"

using Patrol = chapt4_interfaces::srv::Patrol;

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        patrol_service_ = this->create_service<Patrol>("patrol", [&](const Patrol::Request::SharedPtr request,
            Patrol::Response::SharedPtr response) -> void{
                if( (0.0 < request->target_x && request->target_x < 12.0f) &&
                    (0.0 < request->target_y && request->target_y < 12.0f)) {
                    target_x_ = request->target_x;
                    target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS; // success
                    RCLCPP_INFO(this->get_logger(), "Received patrol request: target_x=%.2f, target_y=%.2f", target_x_, target_y_);
                } else {
                    response->result = Patrol::Response::FAIL; // failure
                    RCLCPP_WARN(this->get_logger(), "Received invalid patrol request: target_x=%.2f, target_y=%.2f. Valid range is (0.0, 11.0)", request->target_x, request->target_y);
                }
            });
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControlNode::pose_callback, this, std::placeholders::_1));
    }
private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        auto message = geometry_msgs::msg::Twist();
        double current_x = msg->x;
        double current_y = msg->y;
        RCLCPP_INFO(this->get_logger(), "Current Pose - x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
        // Here you can add logic to control the turtle based on its current pose
        double distance = std::sqrt((target_x_ - current_x) * (target_x_ - current_x) + (target_y_ - current_y) * (target_y_ - current_y));
        double angele = std::atan2(target_y_ - current_y, target_x_ - current_x) - msg->theta;
        if(distance > 0.1) {
            if (fabs(angele) > 0.2) {
                message.angular.z = fabs(angele);
            } else {
                message.linear.x = k_ * distance;
            }
            message.linear.x = 1.0;
            message.angular.z = angele;
        }
        if (message.linear.x > max_speed_) {
            message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);
    }

private:
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};         // 比例系数, 控制输出=误差乘以比例系数 
    double max_speed_{3.0};
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}