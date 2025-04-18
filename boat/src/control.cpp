//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/twist.hpp>
//#include <geometry_msgs/msg/pose.hpp>
//#include <visualization_msgs/msg/marker.hpp>
//#include <vector>
//#include <cmath>
//#include <nav_msgs/msg/odometry.hpp>
//#include <nav_msgs/msg/path.hpp>
//
//using namespace std;
//
//struct State {
//    double x, y, theta;//position
//};
//
//struct Control {//linear and angular velocities
//    double v, omega;
//};
//
//State predict_state(const State& current_state, const Control& control_input, double dt) {// Kinematics for next state
//    State predicted_state = current_state;
//    predicted_state.x += control_input.v * cos(current_state.theta) * dt;  // x' = x + v*cos(theta)*dt
//    predicted_state.y += control_input.v * sin(current_state.theta) * dt;  // y' = y + v*sin(theta)*dt
//    predicted_state.theta += control_input.omega * dt;
//    return predicted_state;
//}
//
//// Cost function (position error + control effort)
//double cost_function(const State& state, const Control& control, const State& target, double control_weight) {
//    double distance = sqrt(pow(state.x - target.x, 2) + pow(state.y - target.y, 2));
//    double control_effort = control.v * control.v + control.omega * control.omega;
//     double angular_deviation = fabs(atan2(target.y - state.y, target.x - state.x) - state.theta);
//
//    return distance + control_weight * control_effort + 2.0 * angular_deviation; // Added angular term
//
//  //  return distance + control_weight * control_effort;//distance + control
//}
//
//Control nmpc_control(const State& current_state, const State& target, double dt, double control_weight) {
//    double learning_rate = 1.0;
//    int max_iterations = 100;
//
//    Control control = { 0.05, 0.05 };//initialization
//
//    for (int iter = 0; iter < max_iterations; ++iter) {
//        State predicted_state = predict_state(current_state, control, dt);
//
//        double cost = cost_function(predicted_state, control, target, control_weight);
//
//        //  gradients
//        double grad_v = (cost_function(predict_state(current_state, { control.v + 0.01, control.omega }, dt),
//            { control.v + 0.01, control.omega }, target, control_weight) - cost) / 0.01;
//        double grad_omega = (cost_function(predict_state(current_state, { control.v, control.omega + 0.01 }, dt),
//            { control.v, control.omega + 0.01 }, target, control_weight) - cost) / 0.01;
//
//        // update controls
//        control.v -= learning_rate * grad_v;
//        control.omega -= learning_rate * grad_omega;
//
//        control.v = max(3.0, min(5.0, control.v));
//        control.omega = max(-M_PI / 4, min(M_PI / 4, control.omega));
//    }
//
//    return control;
//}
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std;

struct State {
    double x, y, theta;//position
};

struct Control {//linear and angular velocities
    double v, omega;
};

State predict_state(const State& current_state, const Control& control_input, double dt) {// Kinematics for next state
    State predicted_state = current_state;
    predicted_state.x += control_input.v * cos(current_state.theta) * dt;  // x' = x + v*cos(theta)*dt
    predicted_state.y += control_input.v * sin(current_state.theta) * dt;  // y' = y + v*sin(theta)*dt
    predicted_state.theta += control_input.omega * dt;
    return predicted_state;
}

// Cost function (position error + control effort)
double cost_function(const State& state, const Control& control, const State& target, double control_weight) {
    double distance = sqrt(pow(state.x - target.x, 2) + pow(state.y - target.y, 2));
    double control_effort = control.v * control.v + control.omega * control.omega;
    double angular_deviation = fabs(atan2(target.y - state.y, target.x - state.x) - state.theta);

    return distance + control_weight * control_effort + 2.0 * angular_deviation; // Added angular term

    //  return distance + control_weight * control_effort;//distance + control
}

Control nmpc_control(const State& current_state, const State& target, double dt, double control_weight) {
    double learning_rate = 1.0;
    int max_iterations = 100;

    Control control = { 0.05, 0.05 };//initialization

    for (int iter = 0; iter < max_iterations; ++iter) {
        State predicted_state = predict_state(current_state, control, dt);

        double cost = cost_function(predicted_state, control, target, control_weight);

        //  gradients
        double grad_v = (cost_function(predict_state(current_state, { control.v + 0.01, control.omega }, dt),
            { control.v + 0.01, control.omega }, target, control_weight) - cost) / 0.01;
        double grad_omega = (cost_function(predict_state(current_state, { control.v, control.omega + 0.01 }, dt),
            { control.v, control.omega + 0.01 }, target, control_weight) - cost) / 0.01;

        // update controls
        control.v -= learning_rate * grad_v;
        control.omega -= learning_rate * grad_omega;

        control.v = max(3.0, min(5.0, control.v));
        control.omega = max(-M_PI / 4, min(M_PI / 4, control.omega));
    }

    return control;
}

class NMPCControllerNode : public rclcpp::Node {
public:
    NMPCControllerNode()
        : Node("boat"), current_state_{ 0.0, 0.0, 0.0 }, dt_(0.05), control_weight_(0.1), waypoint_index_(0) {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        global_path_ = {
            // { 1.0, 1.0, 0.0 },
             { 10.0, 0.0, 0.0 },
     { 20.0, -10.0, 0.0 },
     { -10.0, 20.0, 0.0 },
            {5.0,-10.0,0.0},
        };
        path_msg_.header.frame_id = "map";

        timer_ = this->create_wall_timer(
            chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            [this]() { control_loop(); });

        publish_path_visualization();
    }

private:
    void control_loop() {
        if (waypoint_index_ >= global_path_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Stopping the robot.");
            stop_robot();
            return;
        }

        State target = global_path_[waypoint_index_];//current target

        // current state and target
        RCLCPP_INFO(this->get_logger(), "Current State: x=%.2f, y=%.2f, theta=%.2f | Target: x=%.2f, y=%.2f",
            current_state_.x, current_state_.y, current_state_.theta, target.x, target.y);

        Control control = nmpc_control(current_state_, target, dt_, control_weight_);

        // update state
        current_state_ = predict_state(current_state_, control, dt_);

        auto cmd_vel_msg = geometry_msgs::msg::Twist();   // publishing commands
        cmd_vel_msg.linear.x = control.v;
        cmd_vel_msg.angular.z = control.omega;
        cmd_vel_publisher_->publish(cmd_vel_msg);
        // control.v = 5.0;
        publish_odometry();
        update_path();

        RCLCPP_INFO(this->get_logger(),
            "Control: v=%.2f, omega=%.2f", control.v, control.omega);
        //auto cmd_vel_msg = geometry_msgs::msg::Twist();
        //cmd_vel_msg.linear.x = 0.5;   
        //cmd_vel_msg.angular.z = 0.1; 
        //cmd_vel_publisher_->publish(cmd_vel_msg);

        //RCLCPP_INFO(this->get_logger(), "Publishing constant twist: linear=%.2f, angular=%.2f",
        //    cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);

        // Check if waypoint is reached
        double distance_to_target = sqrt(pow(current_state_.x - target.x, 2) + pow(current_state_.y - target.y, 2));
        if (distance_to_target < 0.3) {  // threshold
            waypoint_index_++;// switch to next point
            RCLCPP_INFO(this->get_logger(), "Waypoint %lu reached. Moving to the next waypoint.", waypoint_index_);
        }
    }

    void stop_robot() {
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }

    void publish_path_visualization() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";// Rviz frame
        marker.header.stamp = now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;


        for (const auto& waypoint : global_path_) {
            geometry_msgs::msg::Point p;
            p.x = waypoint.x;
            p.y = waypoint.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_publisher_->publish(marker);
    }

    void publish_odometry() {//odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = current_state_.x;
        odom_msg.pose.pose.position.y = current_state_.y;
        odom_msg.pose.pose.orientation.z = sin(current_state_.theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(current_state_.theta / 2.0);
        odom_publisher_->publish(odom_msg);
    }

    void update_path() {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = current_state_.x;
        pose_stamped.pose.position.y = current_state_.y;
        pose_stamped.pose.orientation.z = sin(current_state_.theta / 2.0);
        pose_stamped.pose.orientation.w = cos(current_state_.theta / 2.0);
        path_msg_.poses.push_back(pose_stamped);
        path_publisher_->publish(path_msg_);
    }

    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    State current_state_;
    vector<State> global_path_;
    nav_msgs::msg::Path path_msg_;
    size_t waypoint_index_;
    double dt_;
    double control_weight_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCControllerNode>());
    rclcpp::shutdown();
    return 0;
}