# Human Detector for ROS 2

This ROS 2 package utilizes MediaPipe and depth images to detect the position of a human in the x, y, and z coordinates.

The package has been tested with the RealSense **D435i** camera along with the corresponding Gazebo classic plugin.

## Results


![detected human](/images/detected_human.png " ")

Visible point cloud is added only for visualization purposes and it is not part of package.

## Setup

### Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Wiktor-99/human_detector.git
cd ..
rosdep install --from-paths src --ignore-src -r -y

```
### Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running
To start, open a terminal and execute the following command:
```bash
ros2 run human_detector human_detector
```
Next, open a second terminal and enter the following commands:
```bash
ros2 lifecycle set /human_detector configure
ros2 lifecycle set /human_detector activate
```

## Interface

### Input
The node utilizes the following topics as inputs:
- `/camera/depth/image_rect_raw` (Image) - for capturing the RGB image
- `/camera/depth/image_rect_raw` (Image) - for capturing the depth image
- `/camera/depth/camera_info` (CameraInfo) - for capturing camera information

### Constraints
There is one strong limitation rgb image and depth image must be same size.

### Output
The node publishes the following data:
- Transform to detected human
- `/camera/color/person_selected` (Image) - for publishing the image with the detected human

### Parameters
The node uses the following parameters:
- `camera_frame_id` - Name of the frame containing the camera. Default is set to **camera_link**.
- `detected_human_frame_id` - Name of the frame containing the detected human. Default is set to **detected_human**.
- `detected_human_transform_frequency` - Frequency of publishing the transform to the detected human. Default is set to **10Hz**.
- `publish_image_with_detected_human_topic` - Name of the topic for publishing the image with the detected human. Default is set to an empty string. If the topic name is empty, then the image with the selected human will not be published.
