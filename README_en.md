<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SSD for ROS

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#launch-and-usage">Launch and Usage</a></li>
      <li><a href="#parameters">Parameters</a></li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>




<!-- Repository overview -->
## Introduction

This repository provides a package for performing object detection using the Single Shot MultiBox Detector (SSD) in a ROS 2 environment.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- Getting Started -->
## Getting Started

This section describes how to set up this repository.


<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, prepare the following environment before proceeding to the installation steps.

| System | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill    |
| Python | >=3.10              |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation
1. First, navigate to the `src` folder of your ROS 2 workspace.
    ```sh
    cd ~/colcon_ws/src
    ```
2. Clone the ROS package `ssd_nodelet` into the `src` folder.
    ```sh
    git clone -b humble-devel https://github.com/TeamSOBITS/ssd_nodelet.git
    ```
3. Navigate into the cloned repository folder.
    ```sh
    cd ssd_nodelet
    ```
4. Install the required dependencies.
    ```sh
    bash install.sh
    ```
5. Build the package.
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```    
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Launch and Usage -->
## Launch and Usage
Once the package has been successfully built, you can verify its operation using the following steps:

1. Start the camera.

2. In [ssd.launch.py](launch/ssd_ros.launch.py), update **image_topic_name** to match the topic name used by your camera.

    Ex.
    ```python
    default_value="/camera/camera/color/image_raw",           ## realsense
    ```

3. If you are using an RGBD camera, update **point_cloud_topic_name** in [ssd.launch.py](launch/ssd_ros.launch.py) to match the point cloud topic of your camera.

    Ex.
    ```python
    default_value="/camera/camera/depth/color/points",      　## realsense
    ```

4. Depending on whether you want to perform object detection (including people) or face detection, update the following model paths in [ssd.launch.py](launch/ssd_ros.launch.py):
    - voc_object_prototxt_path
    - voc_object_caffemodel_path
    - voc_object_names_path

    Ex. Object Detection:
    ```python
    voc_object_prototxt_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt')
    ```

6. For face detection, in addition to updating the paths, modify **in_scale_factor** in [ssd.launch.py](launch/ssd_ros.launch.py):
    ```python
    # default_value="0.007843",     # For object detection
    default_value="1.00",           # For face detection
    ```


7. Once all the necessary changes are complete, you can launch [ssd.launch.py](launch/ssd_ros.launch.py) to verify that it is working:
    ```sh
    ros2 launch ssd_ros ssd_ros.launch.py
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Parameters
The parameters that can be set in [ssd.launch.py](launch/ssd_ros.launch.py) are as follows:

| Parameter Name  | Description | Default Value |
| ------------- | ------------- | ------------- |
|image_show_flag|Control whether to display images|`true`|
|execute_default|Determines whether SSD (Single Shot MultiBox Detector) starts by default|`true`|
|image_topic_name|Specify the ROS topic name of the `sensor_msgs/msg/Image` message type|`/camera/camera/color/image_raw`（realsense用）|
|point_cloud_topic_name|Specify the ROS topic name of the `sensor_msgs/msg/PointCloud2` message type|`/camera/camera/depth/color/points`（realsense用）|
|in_scale_factor|Scale parameter used for data preprocessing in the Caffemodel|`0.007843`（For object detection）|
|confidence_threshold|Confidence threshold for detection results|`0.5`|
|ssd_prototxt_name|File path describing the structure of the Caffemodel|/install/ssd_ros/share/ssd_ros/models/voc_object.prototxt|
|ssd_caffemodel_name|File path of the trained model|/install/ssd_ros/share/ssd_ros/models/voc_object.caffemodel|
|ssd_class_names_file|File path of the trained object names list|/install/ssd_ros/share/ssd_ros/models/voc_object_names.txt|
|object_specified_enabled|Flag to enable detection of specific objects|`true`|
|specified_object_name|Specify the name of the object to detect when `object_specified_enabled` is set to `true`|`person`|
|use_3d|Flag to enable 3D detection|`true`|
|cluster_tolerance|Threshold for considering how far apart point clusters can be regarded as the same object|`0.01`|
|min_clusterSize|Threshold to reject point clusters below a certain size|`100`|
|max_clusterSize|Threshold to reject point clusters above a certain size|`20000`|
|noise_point_cloud_range|Amount of noise removal from the point cloud of the target object|`0.01`|
|fast_shot|Flag to enable fast_shot|`true`|
|enable_id|Flag to assign IDs to detected object labels|`false`|

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Milestones
* There may be updates to the launch parameters due to changes in `image_to_position`.

Please check the [Issue page][issues-url] for current bugs and feature requests.



<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgments
* [SSD: Single Shot MultiBox Detector](https://www.cs.unc.edu/~wliu/papers/ssd.pdf)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/ssd_nodelet/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/ssd_nodelet/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/ssd_nodelet/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/ssd_nodelet/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/ssd_nodelet.svg?style=for-the-badge
[license-url]: LICENSE
