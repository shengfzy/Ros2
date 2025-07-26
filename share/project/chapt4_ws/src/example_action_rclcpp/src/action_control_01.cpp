#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class ActionControl01 : public rclcpp::Node {
public:
    explicit ActionControl01(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());
    }
private:
};

int main(int argc, char **argv) {
    rclcpp:: init(argc, argv);
    auto node = std::make_shared<ActionControl01>("action_control_01");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

