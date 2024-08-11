#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <ncurses.h>


class KeyboardControlNode : public rclcpp::Node
{
public:
    KeyboardControlNode()
            : Node("keyboard_control_node"),
              linear_speed_(0.0),
              angular_speed_(0.0)
    {
        publisher_diff_drive_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_controller/cmd_vel", 10);
        publisher_camera_position_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        running_.store(true);

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        keyboard_thread_ = std::thread(&KeyboardControlNode::keyboardLoop, this);

        initscr();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        timeout(100);
    }

    ~KeyboardControlNode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        endwin();

        running_.store(false);
        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
    }

private:
    void keyboardLoop()
    {
        while (running_.load())
        {
            int ch = getch();

            switch (ch) {
                case 'w':
                    linear_speed_ = 1.0;
                    break;
                case 's':
                    linear_speed_ = -1.0;
                    break;
                case 'a':
                    angular_speed_ = 1.0;
                    break;
                case 'd':
                    angular_speed_ = -1.0;
                    break;
                case 'q':
                    running_.store(false);
                    std::cout << "Exiting..." << std::endl;
                    break;
                case KEY_UP:
                    camera_angle_ += 0.1;
                    break;
                case KEY_DOWN:
                    camera_angle_ -= 0.1;
                    break;
                case KEY_LEFT:
                    rotator_angle_ += 0.1;
                    break;
                case KEY_RIGHT:
                    rotator_angle_ -= 0.1;
                    break;
                default:
                    linear_speed_ = 0.0;
                    angular_speed_ = 0.0;
                    break;
            }

            publishTwist();
        }
    }

    void publishTwist()
    {
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now();
        twist_stamped.header.frame_id = "base_link";
        twist_stamped.twist.linear.x = linear_speed_;
        twist_stamped.twist.angular.z = angular_speed_;
        std::cout << linear_speed_ << " " << angular_speed_ << std::endl;
        publisher_diff_drive_->publish(twist_stamped);
        auto camera_msg = std_msgs::msg::Float64MultiArray();
        camera_msg.data.resize(2);
        camera_msg.data[0] = camera_angle_;
        camera_msg.data[1] = rotator_angle_;
        publisher_camera_position_->publish(camera_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_diff_drive_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_camera_position_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    struct termios oldt, newt;
    double linear_speed_;
    double angular_speed_;

    double rotator_angle_ = 0.0;
    double camera_angle_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    std::cout << "Keyboard control node started" << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
