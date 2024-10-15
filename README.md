# ROS Package for image processing

This repository contains two ROS package contains having two main nodes: one for publishing images and another for subscribing to an image topic, processing the images by converting them from RGB to grayscale, performing contour analysis, and republishing the processed images.

## System Specifications

### CPU

- **Model:** AMD Ryzen 7 5800H
- **Cores:** 8
- **Threads:** 16
- **Base Clock Speed:** 3.2 GHz
- **Max Boost Clock Speed:** 4.4 GHz
- **Cache:** 16 MB L3 Cache
- **Integrated Graphics:** AMD Radeon Graphics

### Other System Specs

- **GPU:** NVIDIA GeForce RTX 3060
- **RAM:** 16 GB DDR4
- **Storage:** 512 GB SSD

## Project Overview:

### 1. Ros88_pub package (Publisher Node)
- This node publishes images to the `my_camera/image` topic.

### 2. Ros88 package (Subscriber Node)
- Subscribes to the `my_camera/image` topic.
- Converts incoming RGB images to grayscale.
- Performs contour analysis on the grayscale image.
- Republishes the grayscale image with contours overlaid on the `/processed_image`.

## Steps involved in implementation:

1. Creating a master ROS workspace to inhabit two ROS packages.
2. Built the packages
3. Made the launch files
4. Modified the Cmake and package.xml files.
5. Made the node files for subsriber and publisher.
6. Ran the launch files
7. Visualized in Rviz. 

## Implementation Guidelines:

**Publisher Node:**
```mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --
cd ~/ros2_ws
colcon build --packages-select ros88_pub
source install/setup.bash
ros2 launch ros88 ros88_pub_launch.py
```

**Subscriber Node:**
```
cd ~/ros2_ws/src
git clone --
cd ~/ros2_ws
colcon build --packages-select ros88
source install/setup.bash
ros2 launch ros88 ros88_sub_launch.py
```


## Enhancements:

> Mentioned in the report.

# Results:

> Mentioned in the report.
