#ifndef RANGE_SENSOR_BROADCASTER_H
#define RANGE_SENSOR_BROADCASTER_H

#include "controller_interface/controller_interface.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <memory>

namespace range_sensor_broadcaster {
class RangeSensorBroadcaster : public controller_interface::ControllerInterface {
public:
    RangeSensorBroadcaster();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
    // Names should be of form: "joint_name/interface_name"
    std::vector<std::string> interface_names_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> range_pub_;
};
} // namespace range_sensor_broadcaster

#endif