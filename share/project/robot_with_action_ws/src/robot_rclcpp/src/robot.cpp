#include "robot_rclcpp/robot.h"

float Robot::robot_move() {
    int direction = move_distance_ / fabs(move_distance_);
    target_pos_ += move_distance_;

    while(fabs(target_pos_ - current_pos_) > 0.01) {
        float step = fabs(target_pos_ - current_pos_) * direction * 0.1;
        current_pos_ += step;

        std::cout << "Robot move: " << step << "Current pos: " << current_pos_ << std::endl;
    }

    return current_pos_;
}

float Robot::robot_rotate() {
    int direction = rotate_angle_ / fabs(rotate_angle_);

    target_angle_ += rotate_angle_;

    while(fabs(target_angle_ - current_angle_) > 0.01) {
        float step = fabs(target_angle_ - current_angle_) * direction * 0.1;
        current_angle_ += step;

        std::cout << "Robot rotate: " << step << "Current angle: " << current_angle_ << std::endl;
    }

    return current_angle_;
}

float Robot::get_current_pos() {return current_pos_;}
float Robot::get_current_angle() {return current_angle_;}
unsigned int Robot::get_current_status_pos() {return status_pos_;}
unsigned int Robot::get_current_status_angle() {return status_angle_;}

void Robot::stop_rotate() {   
    status_angle_ = robot_interfaces::action::RobotRotate::Feedback::STATUS_STOP;

}


