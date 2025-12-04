# ROS2 visual-inertial ORB-SLAM3 node

There are a couple of nodes that do similar things available online, but I did not find one that matched what I needed. This one is specifically for ROS2, and using the visual-inertial mode of ORB-SLAM3. There is one node for monocular, and one for stereo images.

Made for a Jetson Orin Nano 8GB, which runs Ubuntu 22.04. The ROS2 version used is Humble.

The setup and initial versions of this was based on the following resources, but was extended to support inertial data:
- [Integrating ORB-SLAM3 with ROS2 Humble on Raspberry Pi 5: A Step-by-Step Guide](https://medium.com/@antonioconsiglio/integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361)
- [Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3)

# Instructions

**Note**: Some of these may be specific for building on a Jetson (which is `aarch64`), as it is the platform this is tested on. Your results may vary for other platforms. I will also assume that you have already installed ROS2 humble.

Some of these instructions are based on [Integrating ORB-SLAM3 with ROS2 Humble on Raspberry Pi 5: A Step-by-Step Guide](https://medium.com/@antonioconsiglio/integrating-orb-slam3-with-ros2-humble-on-raspberry-pi-5-a-step-by-step-guide-78e7b911c361).

## Prerequisites

Install Eigen v3.4.0:
```bash
sudo apt install libeigen3-dev
```

Install OpenCV (I used v.4.8.0), either by [building](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) or by installing with your package manager. For a Jetson:
```bash
sudo apt install libopencv
```

Set up cv_bridge (for converting ROS images to OpenCV images). There are versions available for some package managers, but issues may pop up when building this later (see below). I recommend that you build it yourself in your ROS2 workspace, [instructions can be found here](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge).

Set up Pangolin. Start by downloading the source code and installing the recommended prerequisites:
```bash
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
scripts/install_prerequisites.sh recommended
```
For best results, switch to v0.6 which is known to be compatible with ORB-SLAM3:
```bash
git checkout 0.6
```
You should also add a missing include to `include/pangolin/gl/colour.h`:
```c++
#include <limits>
```
Then build and install:
```bash
mkdir build && cd build
cmake .. && make
make install
```
You might have to run `make install` as a superuser depending on your system setup, as it installs shared libraries in `/usr/local/lib` 

Set up ORB-SLAM3. Download the source code, checking out the branch that supports C++14:
```bash
git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git
```
Build ORB-SLAM3:
```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
After building, make sure your system can find the ORB-SLAM3 libraries by adding it to your `LD_LIBRARY_PATH`:
```bash
echo 'export LD_LIBRARY_PATH=<path to ORB-SLAM3>/lib:/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```
Replace `<path to ORB-SLAM3>` with the path to where you downloaded ORB-SLAM3 to.

## Building

Move to the `src` folder your ROS workspace, clone this repository, and move into it:
```bash
cd <path to ros workspace>/src
git clone https://github.com/lovebarany/ros2_vi_slam.git
cd ros2_vi_slam
```
The node is configurable in a couple of ways, some of which need to be set before building (and if they are changed, the node needs to be rebuilt). Move into the configuration folder:
```bash
cd config
```
In `config/generic_topicconfig[stereo,mono].yaml`, you specify the topics that the node listens to, and the topic the node outputs odometry to. This has sensible default values, but change them accordingly if your dataset or sensors outputs data to different topics.

In `config/generic_fileconfig[stereo,mono].yaml`, there are two values that need to be set: the path to the settings file for the sensors you're working with, and the path to the vocabulary file ORB-SLAM3 uses. Begin by copying the generic file to one looked for by the node (untracked by git):
```bash
cp generic_fileconfig[stereo,mono].yaml fileconfig[stereo,mono].yaml
```
And then edit `fileconfig.yaml` accordingly. For instance, if you're testing with the TUM-VI dataset, you could use files provided with ORB-SLAM3 as:
```yaml
/ros2_vi_slam/stereo_slam_sys:
  ros__parameters:
    settings_file: <path to ORB-SLAM3>/Examples/Stereo-Inertial/TUM-VI.yaml
    vocabulary_file: <path to ORB-SLAM3>/Vocabulary/ORBvoc.txt
```
Replace `<path to ORB-SLAM3>` with the path to where you downloaded ORB-SLAM3 to.

All files except for the generic ones in `config/` are untracked by git, as to not litter the repository.

You can also create your own launch files, if you wish to configure the node in some special way, or use some specific configuration file. As in `config/`, all files in `launch/` are untracked except for the generic stereo/mono launch files.

Move back to your root ROS workspace:
```bash
cd <path to ros workspace>
```
Finally, build the node:
```bash
colcon build
```

## Running

While in the root ROS workspace, run the following commands in one terminal, and keep it open:
```bash
source install/local_setup.bash
ros2 launch ros2_vi_slam [stereo,mono]_launch.py enable_window:='True'
```
This should start up the node, and the stereo/mono ORB-SLAM3 system. The `enable_window:='True'` parameter tells the system to open a Pangolin window. To run it headless, simply don't pass the parameter.

Then you simply start providing data to the topics the node expects, for instance using `ros2 bag play` or using some driver node.

By default, when the system is shut down, the trajectory and keyframe trajectory are saved in `/home/<your user>/trajectory-<time node was launched>.txt` and `/home/<your user>/keyframe-trajectory-<time node was launched>.txt`. These can be changed by specifying the `trajectory_file` and `keyframe_trajectory_file` parameters when launching the node, for instance:
```bash
ros2 launch ros2_vi_slam [stereo,mono]_launch.py trajectory_file:='/home/<your user>/traj-1.txt keyframe_trajectory_file:='/home/<your user>/kf-traj-1.txt'
```

# Common Jetson-related problems

## OpenCV versions

When building, you may get warnings related to there existing two versions of OpenCV from `ld`, with cv_bridge expecting a different version than installed. This is most likely due to having installed the cv_bridge package from the apt-repository. As per other solutions online (see [here](https://stackoverflow.com/questions/77901513/conflicting-opencv-versions-with-cv-bridge) for instance), it can be solved by removing the version of cv_bridge installed by your package manager, and building cv_bridge yourself. [Instructions can be found here](https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge).

For best results, you should probably build OpenCV as well: [instructions here](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)

# Not implemented yet/Todos
- [ ] Make it easier to specify vocabulary and settings files, to remove the need to rebuild each time.
- [ ] Move common code from mono/stereo to base node
