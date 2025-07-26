#ifndef EXAMPLE_ACTION_RCLCPP_ROBOT_H
#define EXAMPLE_ACTION_RCLCPP_ROBOT_H
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
public:
    using MoveRobot = robot_control_interfaces::action::MoveRobot;
    Robot() = default;
    ~Robot() = default;

    float move_step();
    bool  set_goal(float distance);
    float get_current_pose();
    int   get_status();
    bool  close_goal();
    void  stop_move();
private:
    float current_pose_ = 0.0;
    float target_pose_ = 0.0;
    float move_distance_ = 0.0;
    std::atomic<bool> cancel_flag_{false};
    int status_ = MoveRobot::Feedback::STATUS_STOP;
};
#endif // EXAMPLE_ACTION_RCLCPP_ROBOT_H
