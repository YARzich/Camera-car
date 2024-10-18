#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <thread>


class ControllerConverter : public rclcpp::Node
{
public:
    ControllerConverter()
            : Node("keyboard_control_node"),
              linear_speed(0.0),
              angular_speed(0.0),
              speed(1.0),
              rotator_angle(0.0),
              camera_angle(0.0),
              adding_angle(0.25)
    {
        publisher_diff_drive = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_controller/cmd_vel", 10);
        publisher_camera_position = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

        subscriber_teleoperation = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/teleoperation_dual_control/commands", 10,
                std::bind(&ControllerConverter::signalConverter, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ControllerConverter started");
    }

private:
    void signalConverter(std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        angular_speed = speed * msg->data[0];
        linear_speed = msg->data[1];

        camera_angle += adding_angle * msg->data[2];
        rotator_angle += adding_angle * msg->data[3];

        publishTwist();
    }

    void publishTwist()
    {
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";
        twist_stamped.twist.linear.x = linear_speed;
        twist_stamped.twist.angular.z = angular_speed;
        publisher_diff_drive->publish(twist_stamped);
        auto camera_msg = std_msgs::msg::Float64MultiArray();
        camera_msg.data.resize(2);
        camera_msg.data[0] = rotator_angle;
        camera_msg.data[1] = camera_angle;
        publisher_camera_position->publish(camera_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_diff_drive;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_camera_position;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_teleoperation;

    rclcpp::TimerBase::SharedPtr timer;

    double linear_speed;
    double angular_speed;
    double speed;

    double rotator_angle;
    double camera_angle;
    double adding_angle;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
