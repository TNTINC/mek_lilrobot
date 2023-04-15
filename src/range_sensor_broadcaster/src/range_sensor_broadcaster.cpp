#include "range_sensor_broadcaster/range_sensor_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace range_sensor_broadcaster {
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

RangeSensorBroadcaster::RangeSensorBroadcaster() : controller_interface::ControllerInterface(){};

CallbackReturn RangeSensorBroadcaster::on_init() { return CallbackReturn::SUCCESS; };

CallbackReturn RangeSensorBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state) {
    this->interface_names_ = {"laser/range"};
    return CallbackReturn::SUCCESS;
};

CallbackReturn RangeSensorBroadcaster::on_activate(const rclcpp_lifecycle::State &previous_state) {
    this->range_pub_ = this->get_node()->create_publisher<sensor_msgs::msg::Range>(
        "laser/range", rclcpp::SensorDataQoS());
    return CallbackReturn::SUCCESS;
};

CallbackReturn RangeSensorBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
    return CallbackReturn::SUCCESS;
};

InterfaceConfiguration RangeSensorBroadcaster::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
};

InterfaceConfiguration RangeSensorBroadcaster::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = interface_names_;
    return state_interfaces_config;
};

return_type RangeSensorBroadcaster::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {

    if (state_interfaces_.size() != 1) {
        RCLCPP_ERROR(get_node()->get_logger(), "RangeSensorBroadcaster: Wrong number of state interfaces");
    }

    double range = state_interfaces_[0].get_value();

    auto range_msg = sensor_msgs::msg::Range();
    range_msg.header.stamp = time;
    range_msg.header.frame_id = "laser";
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = 0.94; // 27*2=54 degrees in radians = 0.94
    range_msg.min_range = 0.000;
    range_msg.max_range = 0.255;

    range_msg.range = range;
    this->range_pub_->publish(range_msg);
};

} // namespace range_sensor_broadcaster

PLUGINLIB_EXPORT_CLASS(
    range_sensor_broadcaster::RangeSensorBroadcaster, controller_interface::ControllerInterface)