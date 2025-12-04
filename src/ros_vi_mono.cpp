#include "ros2_vi_slam/vi_mono_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VIMonoNode>();

    // Track in a separate thread
    std::thread tracking_thread(&VIMonoNode::track, node);

    // Run ROS node
    rclcpp::spin(node);

    rclcpp::shutdown();
    tracking_thread.join();
    std::cout << "ROS2 MONO VI-SLAM DONE" << std::endl;
    return 0;
}
