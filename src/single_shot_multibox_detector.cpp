#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
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

#include <iostream>
#include <fstream>

#include "sobits_msgs/msg/bounding_boxes.hpp"
#include "sobits_msgs/msg/bounding_box.hpp"
#include "sobits_msgs/srv/run_ctrl.hpp"

constexpr size_t RESIZE_WIDTH = 300;
constexpr size_t RESIZE_HEIGHT = 300;
constexpr float  MEAN_VAL = 127.5;//Half of the maximum brightness value

class SSDRos {
    private:
        rclcpp::Node::SharedPtr nd_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
        rclcpp::Publisher<sobits_msgs::msg::BoundingBoxes>::SharedPtr pub_bbox_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
        rclcpp::Service<sobits_msgs::srv::RunCtrl>::SharedPtr run_ctr_srv_;
        std::string topic_name;
        std::string model_configuration_path;
        std::string model_binary_path;
        std::string class_names_file_path;
        std::string specified_object_name_;
        std::vector<std::string> class_names_;
        bool img_show_flag_;
        bool execute_flag_;
        bool object_specified_enabled_;
        cv::dnn::Net net_;
        double confidence_threshold_;
        double in_scale_factor_;
        void read_files() {
            std::ifstream ifs( class_names_file_path.c_str() );
            std::string str;
            class_names_.clear();
            if (ifs.fail()) {
                RCLCPP_ERROR(nd_->get_logger(), "SSD_Object_Detection -> Read File Error");
                rclcpp::shutdown();
                return;
            }
            while (getline(ifs, str)) class_names_.push_back(str);
            RCLCPP_INFO(nd_->get_logger(), "SSD_Object_Detection -> className size = %ld", class_names_.size()-1);
            return;
        }
        void callback_image(const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {
            if (!execute_flag_) return;
            cv::Mat cv_img;
            cv_img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            cv::Mat image_resize;
            cv::resize(cv_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
            cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
            net_.setInput(inputBlob, "data");
            cv::Mat detection = net_.forward("detection_out");
            cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

            sobits_msgs::msg::BoundingBoxes bboxes;
            bboxes.header = img_msg->header;
            for (int i = 0; i < detection_mat.rows; ++i ) {
                float confidence = detection_mat.ptr<float>(i)[2];
                if ( confidence <= confidence_threshold_ ) continue;
                if ( class_names_.size() <= (size_t)(detection_mat.ptr<float>(i)[1]) ) continue;
                if ( object_specified_enabled_ && (class_names_[(size_t)(detection_mat.ptr<float>(i)[1])] != specified_object_name_) ) continue;

                int x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * cv_img.cols);
                int y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * cv_img.rows);
                int x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * cv_img.cols);
                int y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * cv_img.rows);
                cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(x_right_top-x_left_bottom), (int)(y_right_top-y_left_bottom));

                cv::rectangle(cv_img, object_area, cv::Scalar(0, 255, 0) ,2);
                cv::String label = class_names_[(size_t)(detection_mat.ptr<float>(i)[1])] + ": " + std::to_string(confidence);
                int baseLine = 0;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
                cv::rectangle(cv_img, label_rect, cv::Scalar::all(255), cv::FILLED);
                cv::putText(cv_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));

                sobits_msgs::msg::BoundingBox bbox;
                // x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * image_resize.cols);
                // y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * image_resize.rows);
                // x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * image_resize.cols);
                // y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * image_resize.rows);
                // x_left_bottom = static_cast<int>(x_left_bottom * (cv_img.cols / static_cast<float>(RESIZE_WIDTH)));
                // y_left_bottom = static_cast<int>(y_left_bottom * (cv_img.rows / static_cast<float>(RESIZE_HEIGHT)));
                // x_right_top = static_cast<int>(x_right_top * (cv_img.cols / static_cast<float>(RESIZE_WIDTH)));
                // y_right_top = static_cast<int>(y_right_top * (cv_img.rows / static_cast<float>(RESIZE_HEIGHT)));
                bbox.xmin = x_left_bottom;
                bbox.ymin = y_left_bottom;
                bbox.xmax = x_right_top;
                bbox.ymax = y_right_top;
                bbox.probability = confidence;
                bbox.class_name = class_names_[(size_t)(detection_mat.ptr<float>(i)[1])];
                bboxes.bounding_boxes.push_back(bbox);
            }

            if (img_show_flag_) {
                cv::imshow("SSD_Object_Detection Result", cv_img);
                cv::waitKey(1);
            }
            sensor_msgs::msg::Image::SharedPtr pub_image_data = cv_bridge::CvImage(img_msg->header, "bgr8", cv_img).toImageMsg();
            pub_image_->publish(*pub_image_data);
            pub_bbox_->publish(bboxes);
        }
        void callback_RunCtr(const std::shared_ptr<sobits_msgs::srv::RunCtrl::Request> req, std::shared_ptr<sobits_msgs::srv::RunCtrl::Response> res) {
            execute_flag_ = req->request;
            res->response = true;
        }
    public:
        SSDRos(std::shared_ptr<rclcpp::Node> nd): nd_(nd) {
            nd_->declare_parameter("image_topic_name", "/camera/camera/color/image_raw");
            nd_->declare_parameter("ssd_prototxt_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.prototxt");
            nd_->declare_parameter("ssd_caffemodel_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.caffemodel");
            nd_->declare_parameter("ssd_class_names_file", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face_names.txt");
            nd_->declare_parameter("execute_default", true);
            nd_->declare_parameter("image_show_flag", true);
            nd_->declare_parameter("object_specified_enabled", true);
            nd_->declare_parameter("specified_object_name", "None");
            nd_->declare_parameter("confidence_threshold", 0.5);
            nd_->declare_parameter("in_scale_factor", 1.00);
            topic_name = nd_->get_parameter("image_topic_name").as_string();
            model_configuration_path = nd_->get_parameter("ssd_prototxt_name").as_string();;
            model_binary_path = nd_->get_parameter("ssd_caffemodel_name").as_string();
            class_names_file_path = nd_->get_parameter("ssd_class_names_file").as_string();
            execute_flag_ = nd_->get_parameter("execute_default").as_bool();
            img_show_flag_ = nd_->get_parameter("image_show_flag").as_bool();
            object_specified_enabled_ = nd_->get_parameter("object_specified_enabled").as_bool();
            specified_object_name_ = nd_->get_parameter("specified_object_name").as_string();
            confidence_threshold_ = nd_->get_parameter("confidence_threshold").as_double();
            in_scale_factor_ = nd_->get_parameter("in_scale_factor").as_double();

            read_files();
            net_ = cv::dnn::readNetFromCaffe( model_configuration_path, model_binary_path );

            pub_image_ = nd_->create_publisher<sensor_msgs::msg::Image>( "/ssd_ros/detect_result", 1);
            pub_bbox_ = nd_->create_publisher<sobits_msgs::msg::BoundingBoxes>( "/ssd_ros/objects_rect", 1);

            run_ctr_srv_ = nd_->create_service<sobits_msgs::srv::RunCtrl>("/ssd_ros/run_ctr", std::bind(&SSDRos::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));
            sub_image_ = nd_->create_subscription<sensor_msgs::msg::Image>(topic_name, 5, std::bind(&SSDRos::callback_image, this, std::placeholders::_1));
        }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nd = std::make_shared<rclcpp::Node>("single_shot_multibox_detector");
    auto ssd_ros = std::make_shared<SSDRos>(nd);
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}