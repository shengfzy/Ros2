#include "rclcpp/rclcpp.hpp"
#include "example_ros2_interface/msg/robot_status.hpp"
#include "example_ros2_interface/srv/move_robot.hpp"
#include <memory>

class ExampleInterfacesControl : public rclcpp::Node
{
public:
    ExampleInterfacesControl(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());

        /* create move_robot client */
        client_ = this->create_client<example_ros2_interface::srv::MoveRobot>(
            "move_robot"
        );

        /* subscribe robot status topic */
        robot_status_subscriber_ = this->create_subscription<example_ros2_interface::msg::RobotStatus>(
            "robot_status",
            10,
            std::bind(&ExampleInterfacesControl::robot_status_callback_, this, std::placeholders::_1)
        );
    }

    /* Send MoveRobot request */
    void move_robot(float distance){
        RCLCPP_INFO(this->get_logger(), "Request robot move %f", distance);

        while(!client_->wait_for_service(std::chrono::seconds(1))){
            if(!rclcpp::ok()){
                RCLCPP_INFO(this->get_logger(), "Interrupted during wait service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service ready.");
        }

        auto request = std::make_shared<example_ros2_interface::srv::MoveRobot::Request>();
        request->distance = distance;

        client_->async_send_request(
            request,
            std::bind(&ExampleInterfacesControl::result_callback_, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Client<example_ros2_interface::srv::MoveRobot>::SharedPtr client_;
    rclcpp::Subscription<example_ros2_interface::msg::RobotStatus>::SharedPtr robot_status_subscriber_;

    void result_callback_(
        rclcpp::Client<example_ros2_interface::srv::MoveRobot>::SharedFuture result_feature
    ){
        auto response = result_feature.get();
        RCLCPP_INFO(this->get_logger(), "received move result: %f", response->pose);
    }

    void robot_status_callback_(
        const example_ros2_interface::msg::RobotStatus::SharedPtr msg
    ){
        RCLCPP_INFO(this->get_logger(), "received pos: %f, status: %d", msg->pose, msg->status);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleInterfacesControl>("example_interfaces_control_01");

    node->move_robot(5.0);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

