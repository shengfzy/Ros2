#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_move.hpp"
#include "robot_interfaces/action/robot_rotate.hpp"

class Control_01 : public rclcpp::Node {
public:
    explicit Control_01(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());
    }
private:
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control_01>("Control_01");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
