#include "robot_rclcpp/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_move.hpp"
#include "robot_interfaces/action/robot_rotate.hpp"

class Robot_01 : public rclcpp::Node {
public:
    explicit Robot_01(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());
    }
private:
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_01>("Robot_01");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
