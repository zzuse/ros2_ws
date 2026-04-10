#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>
using Patrol = chapt4_interfaces::srv::Patrol;
using namespace std::chrono_literals;


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
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}