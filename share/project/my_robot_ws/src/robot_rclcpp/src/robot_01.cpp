#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_move.hpp"

class Robot {
public:
    Robot() = default;
    ~Robot() = default;

    void robot_move(float distance) {
        status_ = robot_interfaces::msg::RobotStatus::STATUS_MOVING;
        target_pos_ = current_pos_ + distance; 
        while(fabs(target_pos_ - current_pos_) > 0.01) {
            float step = distance / fabs(distance) * fabs(target_pos_ - current_pos_) * 0.1;
            current_pos_ += step;
            std::cout << "moved : " << step << " current pos: " << current_pos_ << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        status_ = robot_interfaces::msg::RobotStatus::STATUS_STOP;
        return;
    }

    float get_current_pos() {return current_pos_;}

    unsigned int get_status() {return status_;}

private:
    float current_pos_ = 0.0;
    float target_pos_ = 0.0;
    unsigned int status_ = robot_interfaces::msg::RobotStatus::STATUS_STOP;
};

class Robot_Node : public rclcpp::Node{
public:
    Robot_Node(std::string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "node %s has been started.", name.c_str());

        publisher_ = this->create_publisher<robot_interfaces::msg::RobotStatus>(
            "RobotStatus", 
            10);
        server_    = this->create_service<robot_interfaces::srv::RobotMove>(
            "RobotMove", 
            std::bind(&Robot_Node::handle_robot_move, this, std::placeholders::_1, std::placeholders::_2));
        timer_     = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Robot_Node::timer_callback, this));
        
        this->declare_parameter("rcl_log_level", 0);
        this->get_parameter("rcl_log_level", log_level_);
        this->get_logger().set_level((rclcpp::Logger::Level)log_level_);

        using namespace std::literals::chrono_literals;
        print_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&Robot_Node::print_timer_callback, this)
        );

    }

private:
    Robot robot_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<robot_interfaces::srv::RobotMove>::SharedPtr server_;
    rclcpp::Publisher<robot_interfaces::msg::RobotStatus>::SharedPtr publisher_;

    int log_level_;
    rclcpp::TimerBase::SharedPtr print_timer_;

    void print_timer_callback() {
        this->get_parameter("rcl_log_level", log_level_);
        this->get_logger().set_level((rclcpp::Logger::Level)log_level_);

        RCLCPP_DEBUG(this->get_logger(), "DEBUG log.");
        RCLCPP_INFO(this->get_logger(), "INFO log.");
        RCLCPP_WARN(this->get_logger(), "WARN log.");
        RCLCPP_ERROR(this->get_logger(), "ERROR log.");
        RCLCPP_FATAL(this->get_logger(), "FATAL log.");
    }

    void timer_callback() {
        robot_interfaces::msg::RobotStatus message;
        message.status = robot_.get_status();
        message.pos = robot_.get_current_pos();

        RCLCPP_INFO(this->get_logger(), "Publishing cur pos = %f.", robot_.get_current_pos());
        publisher_->publish(message);
    }

    void handle_robot_move(
        const std::shared_ptr<robot_interfaces::srv::RobotMove::Request> request,
        std::shared_ptr<robot_interfaces::srv::RobotMove::Response> response
    ) {
        RCLCPP_INFO(this->get_logger(), "received distance: %f, current pos: %f", request->distance, robot_.get_current_pos());
        robot_.robot_move(request->distance);
        response->pos = robot_.get_current_pos();
    }
};

int 
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Robot_Node>("Robot_Node"); 
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
