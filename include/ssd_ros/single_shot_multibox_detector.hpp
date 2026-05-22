#ifndef SINGLE_SHOT_MULTIBOX_DETECTOR_HPP
#define SINGLE_SHOT_MULTIBOX_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <std_srvs/srv/set_bool.hpp>

class SSDRos : public rclcpp::Node {
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_bbox_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;
        std::string topic_name;
        std::string qos_reliability_;
        std::string model_configuration_path;
        std::string model_binary_path;
        std::string class_names_file_path;
        std::vector<std::string> specified_object_classes_;
        std::vector<std::string> class_names_;
        bool img_show_flag_;
        bool object_specified_enabled_;
        cv::dnn::Net net_;
        double confidence_threshold_;
        double in_scale_factor_;
        bool isPath_correct(const std::string& base_path);
        void read_files();
        void callback_image(const std::shared_ptr<sensor_msgs::msg::Image> img_msg);
        void callback_RunCtr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    public:
        SSDRos();
};

#endif // SINGLE_SHOT_MULTIBOX_DETECTOR_HPP