#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define NUM_CIRCLES 1
#define FLIGHT_ALTITUDE -5.0f
#define CIRCLE_RADIUS 1.0f
#define ANGLE_STEP 0.1f

class CustomOffboardControl : public rclcpp::Node
{
    public:
    CustomOffboardControl() : Node("custom_offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);  // 10 = QoS history queue depth
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        // Subscribe to vehicle local position 
        local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            rclcpp::SensorDataQoS(),
            [this](const VehicleLocalPosition::SharedPtr msg) {
                current_position_ = *msg;
            }
        );

        offboard_setpoint_counter_ = 0;
        mission_state_ = 0; // 0: takeoff, 1: circle trajectory, 2: landing
        circle_angle_ = 0.0f;
        circle_radius_ = CIRCLE_RADIUS;
        circle_center_ = {0.0f, 0.0f, FLIGHT_ALTITUDE}; // Circle centered at takeoff position
        takeoff_complete_ = false;
        circles_completed_ = 0;
        land_command_sent_ = false;
        
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm();
            }
            
            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();  
            publish_trajectory_setpoint();
            
            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }
    
    void arm();
    void disarm();
    
    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    
    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;
    int mission_state_;
    float circle_angle_;
    float circle_radius_;
    std::array<float, 3> circle_center_;
    bool takeoff_complete_;
    int circles_completed_;
    bool land_command_sent_;
    VehicleLocalPosition current_position_;
    
    bool reached_position(const std::array<float,3>& target, float tolerance=0.2f);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void CustomOffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void CustomOffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

bool CustomOffboardControl::reached_position(const std::array<float,3>& target, float tolerance) {
    float dx = current_position_.x - target[0];
    float dy = current_position_.y - target[1];
    float dz = current_position_.z - target[2];
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist < tolerance;
}

void CustomOffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void CustomOffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    static std::array<float,3> target{0.0, 0.0, FLIGHT_ALTITUDE};
    const float TWO_PI = 2.0f * M_PI;

    if (offboard_setpoint_counter_ > 10) {
        if (reached_position(target)) {
            if (!takeoff_complete_) {
                // First takeoff complete, start circle trajectory
                takeoff_complete_ = true;
                mission_state_ = 1;
                RCLCPP_INFO(this->get_logger(), "Takeoff complete, starting circle trajectory");
                
                // Set first circle waypoint
                target = {circle_radius_, 0.0f, FLIGHT_ALTITUDE};
            }
            else if (mission_state_ == 1) {
                
                // Check if we completed a full circle
                if (circle_angle_ >= TWO_PI) {
                    circles_completed_++;
                    circle_angle_ = std::fmod(circle_angle_, TWO_PI); // Keep angle in range
                    RCLCPP_INFO(this->get_logger(), "Completed circle %d/%d", circles_completed_, NUM_CIRCLES);

                    // After NUM_CIRCLES circles, start landing
                    if (circles_completed_ >= NUM_CIRCLES) {
                        mission_state_ = 2;
                        target = {0.0, 0.0, FLIGHT_ALTITUDE}; // Return to takeoff position first
                        RCLCPP_INFO(this->get_logger(), "%d circles complete, returning to land", NUM_CIRCLES);
                    }
                }
                else {
                    // Generate next point on circle
                    circle_angle_ += ANGLE_STEP;
                    target[0] = circle_center_[0] + circle_radius_ * std::cos(circle_angle_);
                    target[1] = circle_center_[1] + circle_radius_ * std::sin(circle_angle_);
                    target[2] = circle_center_[2];

                }
            }
            else if (mission_state_ == 2) {
                // Landing sequence: first return to origin at altitude, then descend
                if (target[0] == 0.0f && target[1] == 0.0f && target[2] == FLIGHT_ALTITUDE) {
                    // At origin altitude, now descend
                    target = {0.0, 0.0, 0.0};
                    RCLCPP_INFO(this->get_logger(), "Descending to land");
                } else if (!land_command_sent_) {
                    // Close to ground, send land command once
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                    land_command_sent_ = true;
                    RCLCPP_INFO(this->get_logger(), "Landing command sent");
                }
            }
        }
    }

    msg.position = target;
    msg.yaw = std::nan(""); // NaN tells PX4 to ignore yaw control
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void CustomOffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting custom offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomOffboardControl>());
    rclcpp::shutdown();
    return 0;
}
