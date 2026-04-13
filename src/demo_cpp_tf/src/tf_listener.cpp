#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "chrono"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    TFListener() : Node("tf_listener")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::timer_callback, this));
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            RCLCPP_INFO(this->get_logger(), "Transform from map to base_link: translation(%.2f, %.2f, %.2f), rotation(%.2f, %.2f, %.2f, %.2f)",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z,
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);
            auto translation = transformStamped.transform.translation;
            auto rotation = transformStamped.transform.rotation;
            double roll, pitch, yaw;
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "平移 Translation: x=%.2f, y=%.2f, z=%.2f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "旋转 Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}