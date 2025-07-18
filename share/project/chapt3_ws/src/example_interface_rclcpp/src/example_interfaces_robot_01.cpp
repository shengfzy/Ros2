#include "rclcpp/rclcpp.hpp"
#include "example_ros2_interface/msg/robot_status.hpp"
#include "example_ros2_interface/srv/move_robot.hpp"
#include <chrono>
#include <memory>
#include <thread>

class Robot {
public:
    Robot() = default;
    ~Robot() = default;

    float move_distance(float distance){
        status_ = example_ros2_interface::msg::RobotStatus::STATUS_MOVING;
        target_pose_ += distance;

        while(fabs(target_pose_ - current_pose_) > 0.01){
            float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
            current_pose_ += step;
            std::cout << "moved:" << step << "current pos:" << current_pose_ << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        status_ = example_ros2_interface::msg::RobotStatus::STATUS_STOP;
        return current_pose_;
    }

    float get_current_pose() {return current_pose_;}

    int get_status() {return status_;}
private:
    // current position
    float current_pose_ = 0.0;
    // target position
    float target_pose_ = 0.0;
    int status_ = example_ros2_interface::msg::RobotStatus::STATUS_STOP;
};

class ExampleInterfacesRobot : public rclcpp::Node
{
public:
    ExampleInterfacesRobot(std::string name) : Node(name){
        RCLCPP_INFO(this->get_logger(), "Node %s has been started.", name.c_str());

        /* create move_robot service */
        move_robot_server_ = this->create_service<example_ros2_interface::srv::MoveRobot>(
            "move_robot",
            std::bind(&ExampleInterfacesRobot::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2)
        );

        /* create publisher */
        robot_status_publisher_ = this->create_publisher<example_ros2_interface::msg::RobotStatus>(
            "robot_status",
            10
        );

        /* create timer with 500ms period */
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ExampleInterfacesRobot::timer_callback, this));
         
    }  
private:
    Robot robot;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<example_ros2_interface::srv::MoveRobot>::SharedPtr move_robot_server_;
    rclcpp::Publisher<example_ros2_interface::msg::RobotStatus>::SharedPtr robot_status_publisher_;

    void timer_callback() {
        example_ros2_interface::msg::RobotStatus message;
        message.status = robot.get_status();
        message.pose = robot.get_current_pose();
        RCLCPP_INFO(this->get_logger(), "Publishing: %f", robot.get_current_pose());

        robot_status_publisher_->publish(message);
    }

    void handle_move_robot(
        const std::shared_ptr<example_ros2_interface::srv::MoveRobot::Request> request,
        std::shared_ptr<example_ros2_interface::srv::MoveRobot::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "received request, distance: %f, cur_pos: %f", request->distance, robot.get_current_pose());
        robot.move_distance(request->distance);
        response->pose = robot.get_current_pose();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleInterfacesRobot>("example_interfaces_robot_01");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
