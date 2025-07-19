#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_move.hpp"

class Control_Node : public rclcpp::Node {
public:
    Control_Node(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());

        subscriber_ = this->create_subscription<robot_interfaces::msg::RobotStatus>("RobotStatus", 
                                                                                    10,
                                                                                    std::bind(&Control_Node::robot_status_callback, this, std::placeholders::_1)
                                                                                   );
        client_ = this->create_client<robot_interfaces::srv::RobotMove>("RobotMove");
    }

    void robot_move(float distance) {
        RCLCPP_INFO(this->get_logger(), "request robot move for %f.", distance);

        while(!client_->wait_for_service(std::chrono::seconds(1))) {
            if(!rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Interrruptted during wait for service.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "waiting for service.");
        }

        auto request = std::make_shared<robot_interfaces::srv::RobotMove::Request>();
        request->distance = distance;

        client_->async_send_request(
            request,
            std::bind(&Control_Node::result_robot_move_callback, this, std::placeholders::_1)
        );
    }
private:
    rclcpp::Subscription<robot_interfaces::msg::RobotStatus>::SharedPtr subscriber_;
    rclcpp::Client<robot_interfaces::srv::RobotMove>::SharedPtr client_;

    void robot_status_callback(const robot_interfaces::msg::RobotStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "received pos: %f, status: %d.", msg->pos, msg->status);
    }

    void result_robot_move_callback(
        rclcpp::Client<robot_interfaces::srv::RobotMove>::SharedFuture result_feature
    ) {
        auto response = result_feature.get();
        RCLCPP_INFO(this->get_logger(), "received move result: %f.", response->pos);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control_Node>("Control_Node");

    node->robot_move(5.0);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
