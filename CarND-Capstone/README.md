# CarND-Capstone System Integration Project

## Udacity Self Driving Car Nanodegree Program

This is the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Team

A team was not used for this project, this is an individual submission by Joshua Tillett who can be reached at jtillett86@gmail.com.

## Project Overview

### System Architecture

<img src="graphics/T3_final-project-ros-graph.png" width="100%" />

This diagram depicts the overall system architecture showing the ROS nodes and topics which are used to facilitate communication between the different subsystems of the vehicle.


#### Traffic Light Detection Node

<img src="graphics/T3_final-project-tl-detector.png" width="100%" />

This node takes inputs from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and outputs the locations to stop for the red traffic lights to the `/traffic_waypoint` topic.


#### Waypoint Updater Node

<img src="graphics/T3_final-project-waypoint-updater.png" width="100%" />

This node has inputs from the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics and outputs a list of waypoints ahead of the car with associated target velocities to the `/final_waypoints topic`. The waypoints projected ahead are set at 50 which offers robust performance. The projected waypoints are determined in such a way that slows the vehicle down to a full stop while not exceeding the allowed jerk limit (0.5 m/s^3) if a yellow or red light is detected. If the upcoming traffic light is green or there is no traffic, the vehicle will move forward without exceeding the maximum allowed speed limit.

#### Drive-by-Wire (DBW) Node

<img src="graphics/T3_final-project-dbw-node.png" width="100%" />

When Carla is not being controlled manually, it is controlled autonomously by a Drive-by-Wire (DBW) system which controls the throttle, brake, and steering electronically. The DBW node takes input from the `/current_velocity`, `/twist_cmd`, and `/vehicle/dbw_enabled` topics while publishing throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

#### Traffic Light Detection

This node takes images of traffic lights as input (the /image_color topic) and runs an object detection model on them to determine if they are green, yellow, or red. To create this node, an object detection model had to be trained on traffic light images. A pretrained model was used from the [TensorFlow Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md), then retrained on training data from both the simulator and a collection of real life images gathered from Carla. The simulator provides labels for where traffic lights are on the test track and their associated light status. This was used as the labeling for the model. Because the simulator images are different than the real life images, two separate models were trained for this project, which can be found in the `ros/src/tl_detector/light_classification/models/` folder. The different models are activated based on the launch files in the `ros/src/tl_detector/launch/` folder.

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the "uWebSocketIO Starter Guide" found in the classroom (see Extended Kalman Filter Project lesson).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
