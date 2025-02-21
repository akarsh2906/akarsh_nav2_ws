#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>

class MultiGoal : public rclcpp::Node {
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    MultiGoal() : Node("multi_goal_sender") {
        action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "/follow_waypoints");
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        client_ = this->create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");
        wait_for_service();
    }

    void send_goal() {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        int num_points;
        std::cout << "Enter number of waypoints: ";
        std::cin >> num_points;

        for (int i = 0; i < num_points; i++) {
            float x, y, yaw;
            std::cout << "Enter X for waypoint " << i + 1 << ": ";
            std::cin >> x;
            std::cout << "Enter Y for waypoint " << i + 1 << ": ";
            std::cin >> y;
            std::cout << "Enter Yaw for waypoint " << i + 1 << ": ";
            std::cin >> yaw;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation = createQuaternionFromYaw(yaw);

            waypoints.push_back(pose);
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = waypoints;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&MultiGoal::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending waypoints...");
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void result_callback(const GoalHandleFollow::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Waypoints followed successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to follow waypoints.");
        }
        send_goal();
    }

    void wait_for_service() {
        RCLCPP_INFO(this->get_logger(), "Waiting for /amcl/get_state service to become available...");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available yet, retrying...");
        }
        RCLCPP_INFO(this->get_logger(), "Service is now available!");
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future_result = client_->async_send_request(request,
            std::bind(&MultiGoal::amcl_response_callback, this, std::placeholders::_1));
    }

    void amcl_response_callback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        auto response = future.get();
        if (response->current_state.label == "active") {
            RCLCPP_INFO(this->get_logger(), "AMCL is active! Publishing initial pose...");
            set_initial_pose();
            send_goal();
        } else {
            RCLCPP_WARN(this->get_logger(), "AMCL not active yet, waiting...");
        }
    }

    void set_initial_pose() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = createQuaternionFromYaw(0.0);
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = 0.25;
        msg.pose.covariance[7] = 0.25;
        msg.pose.covariance[35] = 0.06853891909122467;

        RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");
        initial_pose_pub_->publish(msg);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw) {
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        return tf2::toMsg(quaternion);
    }

private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiGoal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
