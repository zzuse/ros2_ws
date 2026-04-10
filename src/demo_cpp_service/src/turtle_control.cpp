#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using Patrol = chapt4_interfaces::srv::Patrol;
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        this->declare_parameter<double>("k", 1.0);
        this->declare_parameter<double>("max_speed", 3.0);
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);
        parameter_callback_handle_ = this->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &params) -> SetParametersResult {
            SetParametersResult result;
            result.successful = true; // Assume success unless we find an invalid parameter
            for (const auto &param : params) {
                if (param.get_name() == "k" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    k_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated parameter k: %.2f", k_);
                } else if (param.get_name() == "max_speed" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    max_speed_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated parameter max_speed: %.2f", max_speed_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid parameter: %s", param.get_name().c_str());
                    result.successful = false; // Mark as unsuccessful if we find an invalid parameter
                    result.reason = "Invalid parameter: " + param.get_name();
                    break; // Exit the loop on the first invalid parameter
                }
            }
            return result;
        });
        patrol_service_ = this->create_service<Patrol>("patrol", [&](const Patrol::Request::SharedPtr request,
            Patrol::Response::SharedPtr response) -> void{
                if( (0.0 < request->target_x && request->target_x < 11.0f) &&
                    (0.0 < request->target_y && request->target_y < 11.0f)) {
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
                message.angular.z = angele;
                message.linear.x = 0.0;
            } else {
                message.linear.x = k_ * distance;
                message.angular.z = angele;
            }
        }
        if (message.linear.x > max_speed_) {
            message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);
    }

private:
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
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