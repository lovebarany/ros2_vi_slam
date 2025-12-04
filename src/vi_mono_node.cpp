#include "ros2_vi_slam/vi_mono_node.hpp"

// https://stackoverflow.com/a/12774387
bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

VIMonoNode::VIMonoNode() : Node("vi_mono_node")
{

    RCLCPP_INFO(this->get_logger(), "\nMONO VI-ORB-SLAM3 node started");

    odom_msg_.header.frame_id = "vi_odom";

    // Set default message values
    odom_msg_.pose.pose.position.x = 0.0;
    odom_msg_.pose.pose.position.y = 0.0;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 0.0;

    /*
     * Declare parameters. All except vocabulary file and settings file
     * are given sensible defaults. Controlled through the launched provided.
     */
    this->declare_parameter("vocabulary_file", "not_set");
    this->declare_parameter("settings_file", "not_set");
    this->declare_parameter("trajectory_file", "not set");
    this->declare_parameter("keyframe_trajectory_file", "not set");
    this->declare_parameter("enable_window", false);
    this->declare_parameter("topic_im", "/cam0/image_raw");
    this->declare_parameter("topic_imu", "/imu0");
    this->declare_parameter("topic_odom", "/odom");

    // Save parameters in corresponding variables

    vocabularyFile = this->get_parameter("vocabulary_file").as_string();
    settingsFile = this->get_parameter("settings_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Vocabulary file set to %s, settings file set to %s",
            vocabularyFile.c_str(), settingsFile.c_str());

    imageTopic = this->get_parameter("topic_im").as_string();
    imuTopic = this->get_parameter("topic_imu").as_string();
    RCLCPP_INFO(this->get_logger(), "Subscribing to topics: image=%s, imu=%s",
            imageTopic.c_str(), imuTopic.c_str());

    odomTopic = this->get_parameter("topic_odom").as_string();
    RCLCPP_INFO(this->get_logger(), "Publishing to topic: odom=%s", odomTopic.c_str());

    // Create subscriptions and publishers, based on topics provided as parameters
    image_ = this->create_subscription<sensor_msgs::msg::Image>(
            imageTopic, 10, std::bind(&VIMonoNode::image_callback, this, std::placeholders::_1));
    imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, 10, std::bind(&VIMonoNode::imu_callback, this, std::placeholders::_1));
    odom_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 10);

    // Make sure vocabulary and settings files actually exist
    if (!fileExists(vocabularyFile) || !fileExists(settingsFile)) {
        RCLCPP_FATAL(this->get_logger(), "Vocabulary and settings files could not be read, make sure they are correct");
        exit(1);
    }

    // Visual-inertial mono tracking 
    ORB_SLAM3::System::eSensor sensorType = ORB_SLAM3::System::IMU_MONOCULAR;
    bool enablePangolinWindow = this->get_parameter("enable_window").as_bool();
    RCLCPP_INFO(this->get_logger(), "Creating visual-inertial mono ORB_SLAM3 system...");
    mpSLAM = std::make_shared<ORB_SLAM3::System>(vocabularyFile, settingsFile, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "Created visual-inertial mono ORB_SLAM3 system.");
}

VIMonoNode::~VIMonoNode()
{
    // Shutdown SLAM system
    RCLCPP_INFO(this->get_logger(), "Shutting down mono ORB_SLAM3 system...");
    mpSLAM->Shutdown();

    // Save trajectory
    std::string trajectoryFile = this->get_parameter("trajectory_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Saving trajectory in %s", trajectoryFile.c_str()); 
    mpSLAM->SaveTrajectoryEuRoC(trajectoryFile);
    // Save keyframe trajectory
    std::string keyFrameTrajectoryFile = this->get_parameter("keyframe_trajectory_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Saving keyframe trajectory in %s", keyFrameTrajectoryFile.c_str()); 
    mpSLAM->SaveKeyFrameTrajectoryEuRoC(keyFrameTrajectoryFile);
}

void
VIMonoNode::image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    std::lock_guard<std::mutex> lock(mBufMutexIm);
    if (!imgBuf.empty())
        imgBuf.pop(); // Remove oldest to process latest
    imgBuf.push(img_msg);
}

void
VIMonoNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double timestamp;
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    // Linear acceleration, in m/s^2
    acc_x = imu_msg->linear_acceleration.x;
    acc_y = imu_msg->linear_acceleration.y;
    acc_z = imu_msg->linear_acceleration.z;
    // Angular velocity, in rad/sec
    gyro_x = imu_msg->angular_velocity.x;
    gyro_y = imu_msg->angular_velocity.y;
    gyro_z = imu_msg->angular_velocity.z;
    // Timestamp, in nanosecs
    timestamp= imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec*1e-9;
    RCLCPP_DEBUG(this->get_logger(), "Recv IMU with stamp: %.9f", timestamp);
    ORB_SLAM3::IMU::Point imuMeas = ORB_SLAM3::IMU::Point(
            acc_x, acc_y, acc_z,
            gyro_x, gyro_y, gyro_z,
            timestamp);
    {
        std::lock_guard<std::mutex> lock(mBufMutexImu);
        imuMeasurements.push_back(imuMeas);
    }
}

cv::Mat
VIMonoNode::getImage(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    // Convert ROS img to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
    return cv_ptr->image.clone();
}

void
VIMonoNode::track()
{
    while (rclcpp::ok())
    {
        cv::Mat im;
        std::vector<ORB_SLAM3::IMU::Point> imuSinceLast;
        double timestampIm;
        if (!imgBuf.empty())
        {
            {
                std::lock_guard<std::mutex> lock(mBufMutexIm);
                im = getImage(imgBuf.front());
                // Timestamp is sent as seconds and nanoseconds, to become seconds.nanoseconds
                timestampIm = imgBuf.front()->header.stamp.sec + imgBuf.front()->header.stamp.nanosec*1e-9;
                RCLCPP_DEBUG(this->get_logger(), "Recv left im with stamp: %.9f", timestampIm);
                imgBuf.pop();
            }

            if (im.empty())
                continue;

            mClahe->apply(im,im);

            {
                std::lock_guard<std::mutex> lock(mBufMutexImu);
                // Copy saved measurements since last images received
                imuSinceLast = imuMeasurements;
                // Remove now old measurements
                imuMeasurements.clear();
            }

            Sophus::SE3f curr_pose;
            try {
                // Track
                curr_pose = mpSLAM->TrackMonocular(im, timestampIm, imuSinceLast);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception caught while trying to track: %s", e.what());
            }

            // publish pose
            publishPose(curr_pose);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void
VIMonoNode::publishPose(const Sophus::SE3f& se3)
{
    Eigen::Vector3f translation = se3.translation();
    odom_msg_.pose.pose.position.x = translation.x();
    odom_msg_.pose.pose.position.y = translation.y();
    odom_msg_.pose.pose.position.z = translation.z();

    Eigen::Matrix3f rotation_matrix = se3.rotationMatrix();
    Eigen::Quaternionf quaternion(rotation_matrix);

    odom_msg_.pose.pose.orientation.x = quaternion.x();
    odom_msg_.pose.pose.orientation.y = quaternion.y();
    odom_msg_.pose.pose.orientation.z = quaternion.z();
    odom_msg_.pose.pose.orientation.w = quaternion.w();

    odom_->publish(odom_msg_);
}
