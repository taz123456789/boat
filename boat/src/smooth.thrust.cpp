// #ifndef M_PI
// #define M_PI 3.14159265358979323846
// #endif
// #include <wiringPi.h>
// #include <softPwm.h>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <vector>
// #include <cmath>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/path.hpp>

// using namespace std;
// #define MOTOR_PIN 2
// #define PWM_RANGE 200
// #define NEUTRAL_PULSE 15
// #define MIN_PULSE 10
// #define MAX_PULSE 20
// #define ARM_DELAY 2000

// // Maximum linear velocity of the boat (m/s)
// const double MAX_LINEAR_VELOCITY = 5.0;

// struct State {
//     double x, y, theta;//position
// };

// struct Control {//linear and angular velocities
//     double v, omega;
// };

// State predict_state(const State& current_state, const Control& control_input, double dt) {// Kinematics for next state
//     State predicted_state = current_state;
//     predicted_state.x += control_input.v * cos(current_state.theta) * dt;  // x' = x + v*cos(theta)*dt
//     predicted_state.y += control_input.v * sin(current_state.theta) * dt;  // y' = y + v*sin(theta)*dt
//     predicted_state.theta += control_input.omega * dt;
//     return predicted_state;
// }

// // Cost function (position error + control effort)
// double cost_function(const State& state, const Control& control, const State& target, double control_weight) {
//     double distance = sqrt(pow(state.x - target.x, 2) + pow(state.y - target.y, 2));
//     double control_effort = control.v * control.v + control.omega * control.omega;
//      double angular_deviation = fabs(atan2(target.y - state.y, target.x - state.x) - state.theta);

//     return distance + control_weight * control_effort + 2.0 * angular_deviation; // Added angular term

//   //  return distance + control_weight * control_effort;//distance + control
// }

// Control nmpc_control(const State& current_state, const State& target, double dt, double control_weight) {
//     double learning_rate = 1.0;
//     int max_iterations = 100;

//     Control control = { 0.05, 0.05 };//initialization

//     for (int iter = 0; iter < max_iterations; ++iter) {
//         State predicted_state = predict_state(current_state, control, dt);

//         double cost = cost_function(predicted_state, control, target, control_weight);

//         //  gradients
//         double grad_v = (cost_function(predict_state(current_state, { control.v + 0.01, control.omega }, dt),
//             { control.v + 0.01, control.omega }, target, control_weight) - cost) / 0.01;
//         double grad_omega = (cost_function(predict_state(current_state, { control.v, control.omega + 0.01 }, dt),
//             { control.v, control.omega + 0.01 }, target, control_weight) - cost) / 0.01;

//         // update controls
//         control.v -= learning_rate * grad_v;
//         control.omega -= learning_rate * grad_omega;

//         control.v = max(-MAX_LINEAR_VELOCITY, min(MAX_LINEAR_VELOCITY, control.v));
//         control.omega = max(-M_PI / 4, min(M_PI / 4, control.omega));
//     }

//     return control;
// }

// int map_thruster_speed(double velocity){
//     return static_cast<int>((velocity/MAX_LINEAR_VELOCITY)*100);
// }
// // setting thruster speed
// void set_thruster_speed(int speedPercent) {
//     if (speedPercent >= -100 && speedPercent <= 100) {
//         double scale = (speedPercent + 100.0) / 200.0;
//         int pulseWidth = static_cast<int>(round(MIN_PULSE + (MAX_PULSE - MIN_PULSE) * scale));
//         softPwmWrite(MOTOR_PIN, pulseWidth);
//         cout << "Setting motor speed to " << speedPercent << "% (Pulse width: " << pulseWidth << ")\n";
//     } else {
//         cout << "Invalid speed! Enter between -100 and +100\n";
//     }
// }

// class NMPCControllerNode : public rclcpp::Node {
// public:
//     NMPCControllerNode()
//         : Node("boat"), current_state_{ 0.0, 0.0, 0.0 }, dt_(0.05), control_weight_(0.1), waypoint_index_(0) {
//         cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
//         marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
//         odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
//         path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

//         global_path_ = {
//            // { 1.0, 1.0, 0.0 },
//             { 10.0, 0.0, 0.0 },
//     { 20.0, -20.0, 0.0 },
//     { -10.0, 30.0, 0.0 },
//         };
//         path_msg_.header.frame_id = "map";

//         timer_ = this->create_wall_timer(
//             chrono::milliseconds(static_cast<int>(dt_ * 1000)),
//             [this]() { control_loop(); });

//         publish_path_visualization();
//     }

// private:
//     void control_loop() {
//         if (waypoint_index_ >= global_path_.size()) {
//             RCLCPP_INFO(this->get_logger(), "All waypoints reached. Stopping the robot.");
//             stop_robot();
//             return;
//         }

//         State target = global_path_[waypoint_index_];//current target

//         // current state and target
//         RCLCPP_INFO(this->get_logger(), "Current State: x=%.2f, y=%.2f, theta=%.2f | Target: x=%.2f, y=%.2f",
//             current_state_.x, current_state_.y, current_state_.theta, target.x, target.y);

//         Control control = nmpc_control(current_state_, target, dt_, control_weight_);

//         // update state
//         current_state_ = predict_state(current_state_, control, dt_);

//         int speedPercent = map_thruster_speed(control.v);
//         set_thruster_speed(speedPercent);

//         auto cmd_vel_msg = geometry_msgs::msg::Twist();   // publishing commands
//         cmd_vel_msg.linear.x = control.v;
//         cmd_vel_msg.angular.z = control.omega;
//         cmd_vel_publisher_->publish(cmd_vel_msg);
//        // control.v = 5.0;
//         publish_odometry();
//         update_path();

//         RCLCPP_INFO(this->get_logger(),
//             "Control: v=%.2f, omega=%.2f", control.v, control.omega);
//         //auto cmd_vel_msg = geometry_msgs::msg::Twist();
//         //cmd_vel_msg.linear.x = 0.5;   
//         //cmd_vel_msg.angular.z = 0.1; 
//         //cmd_vel_publisher_->publish(cmd_vel_msg);

//         //RCLCPP_INFO(this->get_logger(), "Publishing constant twist: linear=%.2f, angular=%.2f",
//         //    cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);

//         // Check if waypoint is reached
//         double distance_to_target = sqrt(pow(current_state_.x - target.x, 2) + pow(current_state_.y - target.y, 2));
//         if (distance_to_target < 0.3) {  // threshold
//             waypoint_index_++;// switch to next point
//             RCLCPP_INFO(this->get_logger(), "Waypoint %lu reached. Moving to the next waypoint.", waypoint_index_);
//         }
//     }

//     void stop_robot() {
//         set_thruster_speed(0); // Stop the thruster
//         auto cmd_vel_msg = geometry_msgs::msg::Twist();
//         cmd_vel_msg.linear.x = 0.0;
//         cmd_vel_msg.angular.z = 0.0;
//         cmd_vel_publisher_->publish(cmd_vel_msg);
//     }

//     void armESC() {
//         softPwmWrite(MOTOR_PIN, NEUTRAL_PULSE);
//         delay(ARM_DELAY);
//         RCLCPP_INFO(this->get_logger(), "ESC Armed!");
//     }

//     void publish_path_visualization() {
//          auto marker = visualization_msgs::msg::Marker();
//         marker.header.frame_id = "map";// Rviz frame
//         marker.header.stamp = now();
//         marker.ns = "path";
//         marker.id = 0;
//         marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.scale.x = 0.1;
//         marker.color.a = 1.0;
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;
     

//         for (const auto& waypoint : global_path_) {
//             geometry_msgs::msg::Point p;
//             p.x = waypoint.x;
//             p.y = waypoint.y;
//             p.z = 0.0;
//             marker.points.push_back(p);
//         }

//         marker_publisher_->publish(marker);
//     }

//     void publish_odometry() {//odometry
//         auto odom_msg = nav_msgs::msg::Odometry();
//         odom_msg.header.stamp = now();
//         odom_msg.header.frame_id = "odom";
//         odom_msg.pose.pose.position.x = current_state_.x;
//         odom_msg.pose.pose.position.y = current_state_.y;
//         odom_msg.pose.pose.orientation.z = sin(current_state_.theta / 2.0);
//         odom_msg.pose.pose.orientation.w = cos(current_state_.theta / 2.0);
//         odom_publisher_->publish(odom_msg);
//     }

//     void update_path() {
//         geometry_msgs::msg::PoseStamped pose_stamped;
//         pose_stamped.header.stamp = now();
//         pose_stamped.header.frame_id = "map";
//         pose_stamped.pose.position.x = current_state_.x;
//         pose_stamped.pose.position.y = current_state_.y;
//         pose_stamped.pose.orientation.z = sin(current_state_.theta / 2.0);
//         pose_stamped.pose.orientation.w = cos(current_state_.theta / 2.0);
//         path_msg_.poses.push_back(pose_stamped);
//         path_publisher_->publish(path_msg_);
//     }

//     //publishers
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     State current_state_;
//     vector<State> global_path_;
//     nav_msgs::msg::Path path_msg_;
//     size_t waypoint_index_;      
//     double dt_;
//     double control_weight_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<NMPCControllerNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <wiringPi.h>
#include <softPwm.h>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

// Thruster control constants
#define MOTOR_PIN 2       // WiringPi pin 16 (Physical pin 26)
#define PWM_RANGE 200     // For 50 Hz with 100 µs steps (20 ms period)
#define NEUTRAL_PULSE 15  // 1.5 ms pulse (15 steps) - Stop position
#define MIN_PULSE 10      // 1.0 ms pulse (10 steps) - Full reverse
#define MAX_PULSE 20      // 2.0 ms pulse (20 steps) - Full forward
#define ARM_DELAY 2000    // Delay for ESC arming (2 seconds)

// State and Control structures
struct State {
    double x, y, theta;  // position and orientation
};

struct Control {
    double v, omega;  // linear and angular velocities
};

// function prototypes
State predict_state(const State& current_state, const Control& control_input, double dt);
double cost_function(const State& state, const Control& control, const State& target, double control_weight);
Control nmpc_control(const State& current_state, const State& target, double dt, double control_weight);
int mapControlToThrust(double control_v, double control_omega);
void sendThrustCommand(int thrustCommand);
void armESC();
Control smooth_control(const Control& prev_control, const Control& new_control, double alpha);
int smooth_thrust_command(int prev_command, int new_command, double alpha);
int limit_rate_of_change(int prev_command, int new_command, int max_change);
int apply_deadband(int command, int deadband);

// NMPC Controller Node
class NMPCControllerNode : public rclcpp::Node {
public:
    NMPCControllerNode()
        : Node("boat"), current_state_{ 0.0, 0.0, 0.0 }, dt_(0.05), control_weight_(0.1), waypoint_index_(0) {
        // initialize WiringPi and thruster
        if (wiringPiSetup() == -1) {
            RCLCPP_ERROR(this->get_logger(), "WiringPi Setup failed!");
            return;
        }
        softPwmCreate(MOTOR_PIN, 0, PWM_RANGE);
        armESC();

        // initialize publishers
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // global path
        global_path_ = {
            { 10.0, 0.0, 0.0 },
            { 20.0, -10.0, 0.0 },
            { -10.0, 20.0, 0.0 },
            { 5.0, -10.0, 0.0 },
        };
        path_msg_.header.frame_id = "map";

        //timer for the control loop
        timer_ = this->create_wall_timer(
            chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            [this]() { control_loop(); });

        //initial path visualization
        publish_path_visualization();
    }

    ~NMPCControllerNode() {
        // stop and clean up
        softPwmWrite(MOTOR_PIN, NEUTRAL_PULSE);
        softPwmStop(MOTOR_PIN);
    }

private:
    void control_loop() {
        if (waypoint_index_ >= global_path_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Stopping the robot.");
            stop_robot();
            return;
        }

        State target = global_path_[waypoint_index_]; 

        // logging current state and target
        RCLCPP_INFO(this->get_logger(), "Current State: x=%.2f, y=%.2f, theta=%.2f | Target: x=%.2f, y=%.2f",
            current_state_.x, current_state_.y, current_state_.theta, target.x, target.y);

        // compute control 
        Control control = nmpc_control(current_state_, target, dt_, control_weight_);

        // Smooth control outputs
        control = smooth_control(prev_control_, control, 0.5);  // change alpha for smoothness
        prev_control_ = control;

        // state update 
        current_state_ = predict_state(current_state_, control, dt_);

        // Map control outputs to command
        int thrustCommand = mapControlToThrust(control.v, control.omega);

        // smooth thrust command
        thrustCommand = smooth_thrust_command(prev_thrust_command_, thrustCommand, 0.5);  // Adjust alpha for smoothness
        thrustCommand = limit_rate_of_change(prev_thrust_command_, thrustCommand, 10);  // Max change of 10%
        thrustCommand = apply_deadband(thrustCommand, 5);  // Deadband of 5%
        prev_thrust_command_ = thrustCommand;

        // send command
        sendThrustCommand(thrustCommand);

        // odometry and update path 
        publish_odometry();
        update_path();

        // log control outputs and thrust command
        RCLCPP_INFO(this->get_logger(),
            "Control: v=%.2f, omega=%.2f | Thrust Command: %d", control.v, control.omega, thrustCommand);

        // check if waypoint is reached
        double distance_to_target = sqrt(pow(current_state_.x - target.x, 2) + pow(current_state_.y - target.y, 2));
        if (distance_to_target < 0.3) {  // Threshold
            waypoint_index_++;  // Switch to next point
            RCLCPP_INFO(this->get_logger(), "Waypoint %lu reached. Moving to the next waypoint.", waypoint_index_);
        }
    }

    void stop_robot() {
        // stop thruster
        sendThrustCommand(0);
    }

    void publish_path_visualization() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";  // Rviz frame
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

    void publish_odometry() {
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

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State and path
    State current_state_;
    vector<State> global_path_;
    nav_msgs::msg::Path path_msg_;
    size_t waypoint_index_;
    double dt_;
    double control_weight_;

    // Smoothing variables
    Control prev_control_ = { 0.0, 0.0 };
    int prev_thrust_command_ = 0;
};

// Helper functions
State predict_state(const State& current_state, const Control& control_input, double dt) {
    State predicted_state = current_state;
    predicted_state.x += control_input.v * cos(current_state.theta) * dt;
    predicted_state.y += control_input.v * sin(current_state.theta) * dt;
    predicted_state.theta += control_input.omega * dt;
    return predicted_state;
}

double cost_function(const State& state, const Control& control, const State& target, double control_weight) {
    double distance = sqrt(pow(state.x - target.x, 2) + pow(state.y - target.y, 2));
    double control_effort = control.v * control.v + control.omega * control.omega;
    double angular_deviation = fabs(atan2(target.y - state.y, target.x - state.x) - state.theta);
    return distance + control_weight * control_effort + 2.0 * angular_deviation;
}

Control nmpc_control(const State& current_state, const State& target, double dt, double control_weight) {
    double learning_rate = 1.0;
    int max_iterations = 100;
    Control control = { 0.05, 0.05 };  // Initialization

    for (int iter = 0; iter < max_iterations; ++iter) {
        State predicted_state = predict_state(current_state, control, dt);
        double cost = cost_function(predicted_state, control, target, control_weight);

        // Gradients
        double grad_v = (cost_function(predict_state(current_state, { control.v + 0.01, control.omega }, dt),
            { control.v + 0.01, control.omega }, target, control_weight) - cost) / 0.01;
        double grad_omega = (cost_function(predict_state(current_state, { control.v, control.omega + 0.01 }, dt),
            { control.v, control.omega + 0.01 }, target, control_weight) - cost) / 0.01;

        // Update controls
        control.v -= learning_rate * grad_v;
        control.omega -= learning_rate * grad_omega;

        // Clamp controls
        control.v = max(1.0, min(5.0, control.v));
        control.omega = max(-M_PI / 4, min(M_PI / 4, control.omega));
    }

    return control;
}

int mapControlToThrust(double control_v, double control_omega) {
    // Map linear velocity (control_v) to thruster speed percentage
    double speedPercent = (control_v / 5.0) * 100.0;

    // Map angular velocity (control_omega) to thruster speed percentage
    double turnPercent = (control_omega / (M_PI / 4)) * 100.0;

    // Combine linear and angular velocities to get the final thrust command
    int thrustCommand = static_cast<int>(speedPercent + turnPercent);

    // Ensure the thrust command is within the valid range [-100, 100]
    thrustCommand = max(-70, min(70, thrustCommand));

    return thrustCommand;
}

void sendThrustCommand(int thrustCommand) {
    // Map thrust command to pulse width
    double scale = (thrustCommand + 100.0) / 200.0;
    int pulseWidth = static_cast<int>(round(MIN_PULSE + (MAX_PULSE - MIN_PULSE) * scale));
    softPwmWrite(MOTOR_PIN, pulseWidth);
}

void armESC() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Arming ESC...");
    softPwmWrite(MOTOR_PIN, NEUTRAL_PULSE);  // Start at neutral position
    delay(ARM_DELAY);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ESC Armed!");
}

Control smooth_control(const Control& prev_control, const Control& new_control, double alpha) {
    Control smoothed;
    smoothed.v = alpha * new_control.v + (1 - alpha) * prev_control.v;
    smoothed.omega = alpha * new_control.omega + (1 - alpha) * prev_control.omega;
    return smoothed;
}

int smooth_thrust_command(int prev_command, int new_command, double alpha) {
    return static_cast<int>(alpha * new_command + (1 - alpha) * prev_command);
}

int limit_rate_of_change(int prev_command, int new_command, int max_change) {
    int change = new_command - prev_command;
    change = max(-max_change, min(max_change, change));
    return prev_command + change;
}

int apply_deadband(int command, int deadband) {
    if (abs(command) < deadband) return 0;
    return command;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCControllerNode>());
    rclcpp::shutdown();
    return 0;
}
