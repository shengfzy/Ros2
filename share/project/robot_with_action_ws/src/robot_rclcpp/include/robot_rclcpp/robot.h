#ifndef ROBOT_RCLCPP_H
#define ROBOT_RCLCPP_H
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/srv/robot_move.hpp"
#include "robot_interfaces/action/robot_rotate.hpp"

class Robot {
public:
    Robot() = default;
    ~Robot() = default;

    float robot_move();  // srv
    float robot_rotate(); // action
    
    unsigned int get_current_status_pos();
    unsigned int get_current_status_angle();
    float get_current_pos();
    float get_current_angle();

    void stop_rotate();

private:
    float current_pos_ = 0.0;
    float target_pos_ = 0.0;
    float move_distance_ = 0.0;

    float current_angle_ = 0.0;
    float target_angle_ = 0.0;
    float rotate_angle_ = 0.0;

    unsigned int status_pos_ = robot_interfaces::msg::RobotStatus::STATUS_STOP;
    unsigned int status_angle_ = robot_interfaces::action::RobotRotate::Feedback::STATUS_STOP;
};
#endif // !ROBOT_RCLCPP_H
