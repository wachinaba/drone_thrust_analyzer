#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"
#include "actuator_control_msgs/srv/actuator_control_target.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

class ActuatorControlPublisher : public rclcpp::Node
{
public:
    ActuatorControlPublisher()
        : Node("actuator_control_publisher")
    {
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
        actuator_servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", 10);
        vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        timer_ = this->create_wall_timer(20ms, std::bind(&ActuatorControlPublisher::timer_callback, this));

        current_control_.resize(4, 0.0);
        target_control_.resize(4, 0.0);

        // サービスの追加
        set_target_control_service_ = this->create_service<actuator_control_msgs::srv::ActuatorControlTarget>(
            "~/set_target_control",
            
            std::bind(&ActuatorControlPublisher::set_target_control_callback, this, std::placeholders::_1, std::placeholders::_2));

        set_arming_service_ = this->create_service<std_srvs::srv::SetBool>(
            "~/set_arming",
            std::bind(&ActuatorControlPublisher::set_arming_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void timer_callback()
    {
        update_control();

        auto ocm_msg = px4_msgs::msg::OffboardControlMode();
        ocm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        ocm_msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(ocm_msg);

        auto am_msg = px4_msgs::msg::ActuatorMotors();
        am_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        for (int i = 0; i < 4; i++) {
            am_msg.control[i] = current_control_[i];
        }
        actuator_motors_publisher_->publish(am_msg);

        auto as_msg = px4_msgs::msg::ActuatorServos();
        as_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        actuator_servos_publisher_->publish(as_msg);
    }

    void update_control()
    {
        // update smoothly
        for (int i = 0; i < 4; i++) {
            current_control_[i] = current_control_[i] * 0.9 + target_control_[i] * 0.1;
            if (abs(current_control_[i] - target_control_[i]) < 0.002) {
                current_control_[i] = target_control_[i];
            }
        }
    }

    // サービスコールバック関数の追加
    void set_target_control_callback(
        const std::shared_ptr<actuator_control_msgs::srv::ActuatorControlTarget::Request> request,
        std::shared_ptr<actuator_control_msgs::srv::ActuatorControlTarget::Response> response)
    {
        if (request->independent) {
            for (int i = 0; i < 4; i++) {
                target_control_[i] = request->values[i];
            }
        } else {
            for (int i = 0; i < 4; i++) {
                target_control_[i] = request->value;
            }
        }
    }

    void set_arming_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        set_offboard_mode();
        set_arming(request->data);
        response->success = true;
    }

    void set_arming(bool arm) {
        request_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, arm ? 1.0 : 0.0);
    }

    void set_offboard_mode() {
        request_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0){
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        request->request = msg;

        auto result = vehicle_command_client_->async_send_request(request, std::bind(&ActuatorControlPublisher::response_callback, this,
                            std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
        auto status = future.wait_for(1s);

        if (status == std::future_status::ready) {
            auto reply = future.get()->reply;
            switch (reply.result) {
                case reply.VEHICLE_CMD_RESULT_ACCEPTED:
                    RCLCPP_INFO(this->get_logger(), "command accepted");
                    break;
                case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                    RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
                    break;
                case reply.VEHICLE_CMD_RESULT_DENIED:
                    RCLCPP_WARN(this->get_logger(), "command denied");
                    break;
                case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
                    RCLCPP_WARN(this->get_logger(), "command unsupported");
                    break;
                case reply.VEHICLE_CMD_RESULT_FAILED:
                    RCLCPP_WARN(this->get_logger(), "command failed");
                    break;
                case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
                    RCLCPP_WARN(this->get_logger(), "command in progress");
                    break;
                case reply.VEHICLE_CMD_RESULT_CANCELLED:
                    RCLCPP_WARN(this->get_logger(), "command cancelled");
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "command reply unknown");
                    break;
                }
        } else {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_publisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<float> current_control_;
    std::vector<float> target_control_;

    rclcpp::Service<actuator_control_msgs::srv::ActuatorControlTarget>::SharedPtr set_target_control_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arming_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorControlPublisher>());
    rclcpp::shutdown();
    return 0;
}

