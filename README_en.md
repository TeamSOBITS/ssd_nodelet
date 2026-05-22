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

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Python | 3.12 |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation
1. First, navigate to the `src` folder of your ROS 2 workspace.
    ```sh
    cd ~/colcon_ws/src
    ```
2. Clone the ROS package `ssd_ros` into the `src` folder.
    ```sh
    git clone -b jazzy-devel https://github.com/TeamSOBITS/ssd_ros.git
    ```
3. Navigate into the cloned repository folder.
    ```sh
    cd ssd_ros
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

2. In [detection_config.yaml](config/detection_config.yaml), update **image_topic_name** to match the topic name used by your camera.

    Ex.
    ```sh
    image_topic_name: "/camera/color/image_raw" # Default Realsense Topic Name
    ```

3. Once all the necessary changes are complete, you can launch [ssd.launch.py](launch/ssd_ros.launch.py) to verify that it is working:
    ```sh
    ros2 launch ssd_ros ssd_ros.launch.py
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Parameters
The parameters that can be set in [ssd.launch.py](launch/ssd_ros.launch.py) and [detection_config.yaml](config/detection_config.yaml) are as follows:

| Parameter Name  | Description | Default Value |
| ------------- | ------------- | ------------- |
|image_show_flag|Control whether to display images|`true`|
|execute_default|Determines whether SSD (Single Shot MultiBox Detector) starts by default|`true`|
|image_topic_name|Specify the ROS topic name of the `sensor_msgs/msg/Image` message type|`/camera/camera/color/image_raw`（realsense用）|
|confidence_threshold|Confidence threshold for detection results|`0.5`|
|specified_object_class| A list of strings specifying the names of objects to detect. If not specified, use `[None]` or `[dummy]`. | [`person`] |
|use_3d|Flag to enable 3D detection|`true`|
|model_directory| The **directory path** containing files describing the Caffe model structure, pre-trained model files, and detailed configuration files for each model. | `../models/objects_model` |



> [!NOTE]
> The `model_directory` should contain the following three items:
> 1. A YAML file containing a list (class) of detectable objects in the order they were trained, and the scale parameters (in_scale_factor) used during data transformation when processing data with Caffemodel
> 2. The file path to the list of trained object names (with the `.caffemodel` extension)
> 3. The file path to the file describing the CaffeModel structure (with the `prototxt` extension)
> Please limit the contents to these three items only.

> [!NOTE]
> For other parameters (especially `bbox_to_3d`), please refer to [image_to_position](https://github.com/TeamSOBITS/image_to_position).
> The focus is primarily on Realsense.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Milestones
* There may be updates to the launch parameters due to changes in `image_to_position`. : Resolved

Please check the [Issue page][issues-url] for current bugs and feature requests.



<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgments
* [SSD: Single Shot MultiBox Detector](https://www.cs.unc.edu/~wliu/papers/ssd.pdf)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/ssd_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/ssd_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/ssd_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/ssd_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/ssd_ros.svg?style=for-the-badge
[license-url]: LICENSE
