#include <transform_to_world_frame/transform_to_world_frame.hpp>

TransformToWorldFrame::TransformToWorldFrame()
    : Node("transform_to_world_frame"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Subscription to original topics
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10,
        std::bind(&TransformToWorldFrame::odometryCallback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 10,
        std::bind(&TransformToWorldFrame::pathCallback, this, std::placeholders::_1));

    // Publishers for transformed topics
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("world_frame/odometry", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("world_frame/global_path", 10);
}

void TransformToWorldFrame::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("world", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform odometry: %s", ex.what());
        return;
    }

    nav_msgs::msg::Odometry transformed_msg = *msg;
    tf2::doTransform(msg->pose.pose, transformed_msg.pose.pose, transform);
    transformed_msg.header.frame_id = "world";

    odom_pub_->publish(transformed_msg);
}

void TransformToWorldFrame::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("world", msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform path: %s", ex.what());
        return;
    }

    nav_msgs::msg::Path transformed_msg = *msg;
    transformed_msg.header.frame_id = "world";

    for (auto &pose_stamped : transformed_msg.poses) {
        tf2::doTransform(pose_stamped.pose, pose_stamped.pose, transform);
    }

    path_pub_->publish(transformed_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformToWorldFrame>());
    rclcpp::shutdown();
    return 0;
}
