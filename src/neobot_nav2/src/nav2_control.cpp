#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>


class Nav2Control : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2Control() : Node("navigate_to_pose_client") {

        // Action client for nav2
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        // InitialPose Publisher and Marker Publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

        // Using this service to check if AMCL is ready
        client_ = this->create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");

        wait_for_service(); //This will also publish initialpose and call send_goal()
    }



    void send_goal() {
        float x, y, yaw;
        std::cout << "Enter goal X: ";
        std::cin >> x;
        std::cout << "Enter goal Y: ";
        std::cin >> y;
        std::cout << "Enter Yaw: ";
        std::cin >> yaw;

        publish_marker(x, y, "yellow");

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation = createQuaternionFromYaw(yaw);


        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&Nav2Control::result_callback, this, std::placeholders::_1, x, y);

        RCLCPP_INFO(this->get_logger(), "Sending goal to (%.2f, %.2f)...", x, y);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void result_callback(const GoalHandleNavigate::WrappedResult &result, float goal_x, float goal_y) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            publish_marker(goal_x, goal_y, "green");
            send_goal();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach goal. Send new goal: ");
            publish_marker(goal_x, goal_y, "red");
            send_goal();
        }
    }


    void wait_for_service() {
        RCLCPP_INFO(this->get_logger(), "Waiting for /amcl/get_state service to become available...");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available yet, retrying...");
        }
        RCLCPP_INFO(this->get_logger(), "Service is now available!");
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future_result = client_->async_send_request(request,
            std::bind(&Nav2Control::amcl_response_callback, this, std::placeholders::_1));
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


    void publish_marker(float x, float y, std::string color) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "goal_markers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        if (color == "green")
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (color == "red")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

