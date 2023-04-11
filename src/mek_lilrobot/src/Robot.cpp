#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mek_lilrobot_interfaces/action/do_mission.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    // Fields
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<rclcpp_action::Client<NavigateToPose>> nav_to_pose;
    std::shared_ptr<rclcpp_action::Server<DoMission>> action_server;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Point>> det_sub;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Empty>> reset_odom_client;
    std::condition_variable nav2_result_cv;
    std::mutex nav2_result_mutex;

public:
    Robot() : Node("robot") {
        // Tf listener to get the current pose
        this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);
        // nav2 NavigateToPose action client
        this->nav_to_pose = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        // wait for nav2 to start
        while (!this->nav_to_pose->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Interrupted while waiting for the action server. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to come up");
        }
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
        RCLCPP_INFO(this->get_logger(), "Got detection: (%f, %f, %f)", msg->x, msg->y, msg->z);
    }

    void nav2_result_callback(
        const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result) {
        RCLCPP_INFO(this->get_logger(), "Result received from nav2");
        std::unique_lock<std::mutex> lk(this->nav2_result_mutex);
        this->nav2_result_cv.notify_all();
    };

    void send_nav2_goal(const geometry_msgs::msg::PoseStamped &goal_pose) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.goal_response_callback = [this](auto) {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by nav2");
        };
        options.result_callback = std::bind(&Robot::nav2_result_callback, this, _1);
        this->nav_to_pose->async_send_goal(goal_msg, options);

        // Wait for the result
        std::unique_lock<std::mutex> lk(this->nav2_result_mutex);
        auto status = this->nav2_result_cv.wait_for(lk, std::chrono::seconds(60));
        if (status == std::cv_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for nav2 result");
        }
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
        orientation *= tf2::Quaternion(tf2::Vector3(0, 0, 1), angle_rad);
        goal_pose.pose.orientation = tf2::toMsg(orientation);
        // Send goal to nav2
        this->send_nav2_goal(goal_pose);
    }

    void drive(double distance) {
        // Get current pose
        auto base_link_to_map_msg =
            this->tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        tf2::Transform base_link_to_map;
        tf2::fromMsg(base_link_to_map_msg.transform, base_link_to_map);

        // Target point in base_link coordinates
        tf2::Transform offset =
            tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(distance, 0, 0));
        // Target point in map coordinates
        tf2::Transform target = base_link_to_map * offset;

        // Create goal pose
        auto target_msg = tf2::toMsg(target);
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();
        goal_pose.pose.position.x = target_msg.translation.x;
        goal_pose.pose.position.y = target_msg.translation.y;
        goal_pose.pose.position.z = target_msg.translation.z;
        goal_pose.pose.orientation = target_msg.rotation;

        // Send goal to nav2
        this->send_nav2_goal(goal_pose);
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
            auto wait_result = wait_set.wait(std::chrono::seconds(5));
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

        // Get detection
        geometry_msgs::msg::Point det;
        rclcpp::MessageInfo info;
        while (rclcpp::ok() && !this->det_sub->take(det, info))
            ;
        // Turn to face detection
        RCLCPP_INFO(this->get_logger(), "Turning to face detection with x: %f", det.x);
        while (fabs(det.x) > 2 / 480.0) {
            auto error = det.x;
            // Approximate turn required at deviation = 1 is horizontal fov/2 = ~27° = ~0.47 rad
            // Underestimating the turn required is better than overestimating to avoid oscillation
            // Negative error means the detection is to the left, so turn left (positive turn)
            auto turn = -error * 0.47 * 0.95;
            this->turn(turn);

            // Get new detection
            while (rclcpp::ok() && !this->det_sub->take(det, info))
                ;
        }

        // Drive towards detection until it is in the lower quarter of the frame
        while (rclcpp::ok() && det.y < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Approaching detection with y: %f", det.y);
            this->drive(0.1);
            // Get new detection
            while (rclcpp::ok() && !this->det_sub->take(det, info))
                ;
        }

        RCLCPP_INFO(this->get_logger(), "End of execute()");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}