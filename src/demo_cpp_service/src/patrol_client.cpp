#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>
#include "rcl_interfaces/msg/parameter.h"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;
using SetP = rcl_interfaces::srv::SetParameters;

class PatrolClientNode : public rclcpp::Node
{
public:
    PatrolClientNode() : Node("patrol_client_node")
    {
        srand(time(NULL));
        patrol_client = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(10s, [&]()->void{
            while (!patrol_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
            }

            auto request = std::make_shared<Patrol::Request>();
            request->target_x = static_cast<float>(rand() % 10 + 1); // 生成1.0到10.0之间的随机数
            request->target_y = static_cast<float>(rand() % 10 + 1); // 生成1.0到10.0之间的随机数
            RCLCPP_INFO(this->get_logger(), "Sending patrol request: target_x=%.2f, target_y=%.2f", request->target_x, request->target_y);
            patrol_client->async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Patrol succeeded");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Patrol failed");
                }
            });
             // 等待结果
        });
    }

    SetP::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &param)
    {
        auto param_client = this->create_client<SetP>("/turtle_control_node/set_parameters");
        while (!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(param);

        auto result_future = param_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            return result_future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service set_parameters");
            return nullptr;
        }
    }

    void update_server_param_k(double k) {
        rcl_interfaces::msg::Parameter param;
        param.name = "k";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = k;

        auto response = call_set_parameter(param);
        if (response == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Failed to update parameter k");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully updated parameter k to %.2f", k);
        }
        for(auto result:response->results) {
            if (!result.successful) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update parameter k: %s", result.reason.c_str());
            } else { 
                RCLCPP_INFO(this->get_logger(), "Parameter k updated successfully");
            }
        }
    }

private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClientNode>();
    node->update_server_param_k(4.0); // 更新服务器参数k的值为1.5
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}