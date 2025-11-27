#include "ros2_vi_slam/vi_stereo_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VIStereoNode>();

    // Track in a separate thread
    std::thread tracking_thread(&VIStereoNode::track, node);

    // Run ROS node
    rclcpp::spin(node);

    rclcpp::shutdown();
    tracking_thread.join();
    std::cout << "ROS2 VI-STEREO SLAM DONE" << std::endl;
    return 0;
}
