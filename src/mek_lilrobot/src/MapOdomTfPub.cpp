#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

/* Todo
 * [X] Publish map -> odom transform
 * [X] Allow resetting map->odom transform
 */
using namespace std::placeholders;
class MapOdomTfPub : public rclcpp::Node {
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using Empty_Response = std_srvs::srv::Empty_Response;
    using Empty_Request = std_srvs::srv::Empty_Request;

    // Fields
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> reset_odom_srv;

    public:
    MapOdomTfPub() : Node("map_odom_tf_pub") {
        // map -> odom transform broadcaster
        auto qos_profile = rmw_qos_profile_default;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile);
        tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this, qos);

        // Send initial 0,0,0 0,0,0,0 map->odom transform
        auto init_tf = TransformStamped();
        init_tf.header.stamp = this->get_clock()->now();
        init_tf.header.frame_id = "map";
        init_tf.child_frame_id = "odom";
        tf_broadcaster->sendTransform(init_tf);

        // transform listener to get odom and base_link
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Service to set map->odom such that map->base_link = 0,0,0 0,0,0,0
        reset_odom_srv = this->create_service<std_srvs::srv::Empty>(
            "/robot/reset_odom", std::bind(&MapOdomTfPub::reset_odom, this, _1, _2, _3));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node constructor finished");
    }

    private:
    // Service handler, sets the map->odom transform such that map->base_link = 0,0,0 0,0,0,0
    void reset_odom(
        const std::shared_ptr<rmw_request_id_t> hdr,
        const std::shared_ptr<Empty_Request> req,
        std::shared_ptr<Empty_Response> res
        ){ 
        (void)hdr; (void)req; (void)res;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset odom service callback");

        tf2::Transform odom_to_base_link{};
        tf2::fromMsg(
            tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero).transform, 
            odom_to_base_link);

        auto map_to_odom = odom_to_base_link.inverse();

        auto map_to_odom_msg = TransformStamped();
        map_to_odom_msg.header.stamp = this->get_clock()->now();
        map_to_odom_msg.header.frame_id = "map";
        map_to_odom_msg.child_frame_id = "odom";
        map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
        this->tf_broadcaster->sendTransform(map_to_odom_msg);
    }

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomTfPub>());
  rclcpp::shutdown();
  return 0;
}