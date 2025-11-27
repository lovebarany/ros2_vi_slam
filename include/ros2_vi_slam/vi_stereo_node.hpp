#ifndef VI_STEREO_NODE_HPP
#define VI_STEREO_NODE_HPP

// Base ROS2 include
#include "rclcpp/rclcpp.hpp"

// Messages used
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

// OpenCV includes
#include <cv_bridge/cv_bridge.h> // Needed to convert ROS2 objects->OpenCV objects
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "System.h"  // Include the SLAM system header

#include <queue>
#include <mutex>
#include <memory>
#include <string>

class VIStereoNode : public rclcpp::Node
{
    public:
            VIStereoNode(); // Constructor
            ~VIStereoNode(); // Destructor
            void virtual track(); // Check for new images, and track with these and IMU measurements saved

    private:
            std::mutex mBufMutexLeft, mBufMutexRight, mBufMutexImu; // Mutexes for access to image and IMU queues/vectors
            std::queue<sensor_msgs::msg::Image::SharedPtr> imgBufLeft, imgBufRight; // Queues to contain incoming camera images
            std::vector<ORB_SLAM3::IMU::Point> imuMeasurements; // Vector of IMU measurements since last camera frame

            void imageLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg); // Callback to save image sent on left channel
            void imageRight_callback(const sensor_msgs::msg::Image::SharedPtr msg); // Callback to save image sent on right channel
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); // Callback to save IMU measurement sent
            cv::Mat getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg); // Convert ROS image to CV image, expected by ORB_SLAM3

            void publishPose(const Sophus::SE3f& se3); // Publish tracked pose on odom_pub_

            std::string vocabularyFile = "";
            std::string settingsFile = "";
            std::shared_ptr<ORB_SLAM3::System> mpSLAM; // Shared pointer to ORB_SLAM3 system, used for visual-inertial tracking

            cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // CLAHE object, image equalization
            
            std::string odomTopic = "";
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_; // Publisher
            nav_msgs::msg::Odometry odom_msg_; // Message to publish

            std::string imageLeftTopic = "";
            std::string imageRightTopic = "";
            std::string imuTopic = "";
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageLeft_, imageRight_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
};

#endif // VI_STEREO_NODE_HPP
