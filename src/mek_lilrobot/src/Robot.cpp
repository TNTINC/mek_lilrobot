#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mek_lilrobot_interfaces/action/do_mission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

/*  Todo
 *  [X] Search state, turn until there is a detection
        [X] Get current pose (map->base_link tf)
 *  [ ] Aquire state, drive forwards until close enough to grab
 *      [ ] Driver for the laser sensor
 *  [ ] Return state, drive to preprogrammed map point depending on color
 */

using namespace std::placeholders;
class Robot : public rclcpp::Node {
    using DoMission = mek_lilrobot_interfaces::action::DoMission;
    using GoalHandle = rclcpp_action::ServerGoalHandle<DoMission>;
    using GoalResponse = rclcpp_action::GoalResponse;
    using CancelResponse = rclcpp_action::CancelResponse;

    // Fields
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> goal_pose_pub;
    std::shared_ptr<rclcpp_action::Server<DoMission>> action_server;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Point>> det_sub;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> reset_odom_client;

public:
    Robot() : Node("robot") {
        // Tf listener to get the current pose
        this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);
        // /goal_pose publisher to send the goal pose
        this->goal_pose_pub =
            this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
        // subcriber to get detection info
        this->det_sub = this->create_subscription<geometry_msgs::msg::Point>(
            "det_deviation", 1, std::bind(&Robot::det_callback, this, _1));
        // Action server to execute the mission
        this->action_server = rclcpp_action::create_server<DoMission>(
            this,
            "do_mission",
            std::bind(&Robot::handle_goal, this, _1, _2),
            std::bind(&Robot::handle_cancel, this, _1),
            std::bind(&Robot::handle_accepted, this, _1));
        // Client to reset odometry
        this->reset_odom_client = this->create_client<std_srvs::srv::Empty>("/robot/reset_odom");
    }

private:
    auto handle_goal(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const DoMission::Goal> goal)
        -> GoalResponse {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Starting mission");
        return GoalResponse::ACCEPT_AND_EXECUTE;
    }

    auto handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) -> CancelResponse {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Canceling mission");
        return CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        std::thread{std::bind(&Robot::execute, this, _1), goal_handle}.detach();
    }

    void det_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Got detection");
    }

    void turn(double angle_rad) {
        // Get current pose
        auto current_pose =
            this->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        // Create goal pose
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.pose.position.x = current_pose.transform.translation.x;
        goal_pose.pose.position.y = current_pose.transform.translation.y;
        goal_pose.pose.position.z = current_pose.transform.translation.z;
        // Turn
        tf2::Quaternion orientation;
        tf2::fromMsg(current_pose.transform.rotation, orientation);
        RCLCPP_INFO(this->get_logger(), "Current angle: %f", orientation.getAngle());
        orientation *= tf2::Quaternion(tf2::Vector3(0, 0, 1), angle_rad);
        RCLCPP_INFO(this->get_logger(), "New angle: %f", orientation.getAngle());
        goal_pose.pose.orientation = tf2::toMsg(orientation);

        // Publish goal pose
        this->goal_pose_pub->publish(goal_pose);
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing mission");
        auto feedback = std::make_shared<DoMission::Feedback>();
        auto result = std::make_shared<DoMission::Result>();

        // Reset odometry
        RCLCPP_INFO(this->get_logger(), "Resetting odometry");
        auto odom_res = reset_odom_client->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>());
        odom_res.wait();

        // Enter "Search" state
        RCLCPP_INFO(this->get_logger(), "Entering search state");
        feedback->state = "search";
        goal_handle->publish_feedback(feedback);

        // Turn until there is a detection
        auto wait_set = rclcpp::WaitSet({{this->det_sub}});
        int turn_counter = 0;
        while (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Awaiting detection");
            auto wait_result = wait_set.wait(std::chrono::seconds(10));
            if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
                break;
            } else if (turn_counter++ > 3) {
                RCLCPP_WARN(this->get_logger(), "No detection, aborting");
                goal_handle->abort(result);
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "Still searching, turning");
                this->turn(M_PI / (2 * 5));
            }
        }

        // Enter "Approach" state
        RCLCPP_INFO(this->get_logger(), "Entering approach state");
        feedback->state = "approach";
        goal_handle->publish_feedback(feedback);

        


    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}