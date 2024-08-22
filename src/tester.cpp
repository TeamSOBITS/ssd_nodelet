#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <limits>
#include <math.h>
#include <cmath>
#include <string>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

// #include "human_feature_detection/point_cloud_processor.hpp"
// #include "sobits_msgs/msg/feature3d.hpp"

constexpr size_t RESIZE_WIDTH = 300;
constexpr size_t RESIZE_HEIGHT = 300;
constexpr float  MEAN_VAL = 127.5;//Half of the maximum brightness value

class TEST {
    private:
        rclcpp::Node::SharedPtr nd_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_points_;
        std::string topic_name;
        std::string model_configuration_path;
        std::string model_binary_path;
        std::string class_names_file_path;
        cv::dnn::Net net_;
        double in_scale_factor_;
        void callback_image(const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {
            in_scale_factor_ = 1.000;
            cv::Mat input_img;
            input_img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            cv::Mat image_resize;
            cv::resize(input_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
            cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
            net_.setInput(inputBlob, "data");
            cv::Mat detection = net_.forward("detection_out");
            cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

            int detect_num = detection_mat.rows;
            double confidence_threshold = 0.5;

            for (int i = 0; i < detect_num; ++i ) {
                float confidence = detection_mat.ptr<float>(i)[2];
                // RCLCPP_INFO(nd_->get_logger(), "confidence1 : %d\n", i);
                if ( confidence <= confidence_threshold ) continue;
                RCLCPP_INFO(nd_->get_logger(), "a : %ld\n", (size_t)(detection_mat.ptr<float>(i)[1]));
                if ( (size_t)(detection_mat.ptr<float>(i)[1]) != 1 ) continue;
                // RCLCPP_INFO(nd_->get_logger(), "confidence2 : %f\n", confidence);

                int x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * input_img.cols);
                int y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * input_img.rows);
                int x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * input_img.cols);
                int y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * input_img.rows);
                cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(x_right_top-x_left_bottom), (int)(y_right_top-y_left_bottom));

                cv::rectangle(input_img, object_area, cv::Scalar(0, 255, 0) ,2);
                cv::String label = "face: " + std::to_string(confidence);
                int baseLine = 0;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
                cv::rectangle(input_img, label_rect, cv::Scalar::all(255), cv::FILLED);
                cv::putText(input_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));
            }

            cv::imshow("SSD_Object_Detection Result", input_img);
            cv::waitKey(1);
        }
    public:
        TEST(std::shared_ptr<rclcpp::Node> nd): nd_(nd) {
            nd_->declare_parameter("topic_name", "/camera/camera/color/image_raw");
            nd_->declare_parameter("ssd_prototxt_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.prototxt");
            nd_->declare_parameter("ssd_caffemodel_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.caffemodel");
            nd_->declare_parameter("ssd_class_names_file", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.txt");
            topic_name = nd_->get_parameter("topic_name").as_string();
            model_configuration_path = nd_->get_parameter("ssd_prototxt_name").as_string();;
            model_binary_path = nd_->get_parameter("ssd_caffemodel_name").as_string();
            class_names_file_path = nd_->get_parameter("ssd_class_names_file").as_string();
            net_ = cv::dnn::readNetFromCaffe( model_configuration_path, model_binary_path );
            sub_points_ = nd_->create_subscription<sensor_msgs::msg::Image>(topic_name, 5, std::bind(&TEST::callback_image, this, std::placeholders::_1));
        }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nd = std::make_shared<rclcpp::Node>("ssd_test");
    auto test = std::make_shared<TEST>(nd);
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}