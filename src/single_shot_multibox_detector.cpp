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

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"

#include <std_srvs/srv/set_bool.hpp>

constexpr size_t RESIZE_WIDTH = 300;
constexpr size_t RESIZE_HEIGHT = 300;
constexpr float  MEAN_VAL = 127.5;//Half of the maximum brightness value

class SSDRos : public rclcpp::Node {
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_bbox_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctr_srv_;
        std::string topic_name;
        std::string model_configuration_path;
        std::string model_binary_path;
        std::string class_names_file_path;
        std::string specified_object_name_;
        std::vector<std::string> class_names_;
        bool img_show_flag_;
        bool object_specified_enabled_;
        cv::dnn::Net net_;
        double confidence_threshold_;
        double in_scale_factor_;
        void read_files() {
            std::ifstream ifs( class_names_file_path.c_str() );
            std::string str;
            class_names_.clear();
            if (ifs.fail()) {
                RCLCPP_ERROR(this->get_logger(), "SSD_Object_Detection -> Read File Error");
                rclcpp::shutdown();
                return;
            }
            while (getline(ifs, str)) class_names_.push_back(str);
            RCLCPP_INFO(this->get_logger(), "SSD_Object_Detection -> className size = %ld", class_names_.size()-1);
            return;
        }
        void callback_image(const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {
            cv::Mat cv_img;
            cv_img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            cv::Mat image_resize;
            cv::resize(cv_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
            cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
            net_.setInput(inputBlob, "data");
            cv::Mat detection = net_.forward("detection_out");
            cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

            vision_msgs::msg::Detection2DArray detection_array_msg;
            detection_array_msg.header = img_msg->header;
            for (int i = 0; i < detection_mat.rows; ++i ) {
                float confidence = detection_mat.ptr<float>(i)[2];
                if ( confidence <= confidence_threshold_ ) continue;
                if ( class_names_.size() <= (size_t)(detection_mat.ptr<float>(i)[1]) ) continue;
                if ( object_specified_enabled_ && (class_names_[(size_t)(detection_mat.ptr<float>(i)[1])] != specified_object_name_) ) continue;

                float x_left_bottom = static_cast<float>(detection_mat.ptr<float>(i)[3] * cv_img.cols);
                float y_left_bottom = static_cast<float>(detection_mat.ptr<float>(i)[4] * cv_img.rows);
                float x_right_top = static_cast<float>(detection_mat.ptr<float>(i)[5] * cv_img.cols);
                float y_right_top = static_cast<float>(detection_mat.ptr<float>(i)[6] * cv_img.rows);

                // 幅
                float width = static_cast<float>(x_right_top - x_left_bottom);
                float hight = static_cast<float>(y_right_top - y_left_bottom);
                // 中点
                float x_half = static_cast<float>((x_right_top + x_left_bottom)/2);
                float y_half = static_cast<float>((y_right_top + y_left_bottom)/2);

                // 描画
                cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(width), (int)(hight));
                cv::rectangle(cv_img, object_area, cv::Scalar(0, 255, 0) ,2);
                cv::String label = class_names_[(size_t)(detection_mat.ptr<float>(i)[1])] + ": " + std::to_string(confidence);
                int baseLine = 0;
                cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
                cv::rectangle(cv_img, label_rect, cv::Scalar::all(255), cv::FILLED);
                cv::putText(cv_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));

                // Detection2D型の設定
                vision_msgs::msg::Detection2D content;
                // ヘッダーの追加
                content.header = img_msg->header;

                vision_msgs::msg::ObjectHypothesisWithPose ohwp;
                // オブジェクト名
                ohwp.hypothesis.class_id = class_names_[(size_t)(detection_mat.ptr<float>(i)[1])];
                // 信頼度
                ohwp.hypothesis.score = confidence;
                // // 位置情報
                // ohwp.pose.pose.position.x = 0.0;
                // ohwp.pose.pose.position.y = 0.0;
                // ohwp.pose.pose.position.z =  0.0;
                // // クォータニオン
                // ohwp.pose.pose.orientation.x = 0.0;
                // ohwp.pose.pose.orientation.y = 0.0;
                // ohwp.pose.pose.orientation.z = 0.0;
                // ohwp.pose.pose.orientation.w = 1.0;
                // ohwp.pose.covariance.fill(0.0);
                // results配列に追加
                content.results.push_back(ohwp);

                // BoundingBox2D型の設定
                vision_msgs::msg::BoundingBox2D bbox;
                // 中心座標
                bbox.center.position.x = x_half;
                bbox.center.position.y = y_half;
                // 回転
                bbox.center.theta = 0.0;
                // 幅・高さ
                bbox.size_x = width;
                bbox.size_y = hight;
                // Detection2D型のbboxにバウンディグボックスの情報を格納
                content.bbox = bbox;

                content.id = class_names_[(size_t)(detection_mat.ptr<float>(i)[1])];

                detection_array_msg.detections.push_back(content);
            }

            if (img_show_flag_) {
                cv::imshow("SSD_Object_Detection Result", cv_img);
                cv::waitKey(1);
            }
            sensor_msgs::msg::Image::SharedPtr pub_image_data = cv_bridge::CvImage(img_msg->header, "bgr8", cv_img).toImageMsg();
            pub_image_->publish(*pub_image_data);
            pub_bbox_->publish(detection_array_msg);
        }
        void callback_RunCtr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            rclcpp::QoS qos_profile(5); // depth = 5
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            // qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

            if (req->data) {
                if (!sub_image_) sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(topic_name, qos_profile, std::bind(&SSDRos::callback_image, this, std::placeholders::_1));
            } else {
                if (sub_image_)  sub_image_.reset();
            }
            res->success = true;
        }
    public:
        SSDRos(): Node("single_shot_multibox_detector") {
            this->declare_parameter("image_topic_name", "/camera/camera/color/image_raw");
            this->declare_parameter("ssd_prototxt_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.prototxt");
            this->declare_parameter("ssd_caffemodel_name", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face.caffemodel");
            this->declare_parameter("ssd_class_names_file", "/home/sobits/colcon_ws/src/ssd_nodelet/models/face_names.txt");
            this->declare_parameter("execute_default", true);
            this->declare_parameter("image_show_flag", true);
            this->declare_parameter("object_specified_enabled", true);
            this->declare_parameter("specified_object_name", "None");
            this->declare_parameter("confidence_threshold", 0.5);
            this->declare_parameter("in_scale_factor", 1.00);
            topic_name = this->get_parameter("image_topic_name").as_string();
            model_configuration_path = this->get_parameter("ssd_prototxt_name").as_string();;
            model_binary_path = this->get_parameter("ssd_caffemodel_name").as_string();
            class_names_file_path = this->get_parameter("ssd_class_names_file").as_string();
            img_show_flag_ = this->get_parameter("image_show_flag").as_bool();
            object_specified_enabled_ = this->get_parameter("object_specified_enabled").as_bool();
            specified_object_name_ = this->get_parameter("specified_object_name").as_string();
            confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
            in_scale_factor_ = this->get_parameter("in_scale_factor").as_double();

            read_files();
            net_ = cv::dnn::readNetFromCaffe( model_configuration_path, model_binary_path );

            pub_image_ = this->create_publisher<sensor_msgs::msg::Image>( "/ssd_ros/detect_result", 1);
            pub_bbox_ = this->create_publisher<vision_msgs::msg::Detection2DArray>( "/ssd_ros/objects_rect", 1);

            run_ctr_srv_ = this->create_service<std_srvs::srv::SetBool>("run_ctr", std::bind(&SSDRos::callback_RunCtr, this, std::placeholders::_1, std::placeholders::_2));

            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
            request->data = this->get_parameter("execute_default").as_bool();
            callback_RunCtr(request, response);

            if (!response->success) RCLCPP_ERROR(this->get_logger(), "Failed to start processing at initialization.");
        }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nd = std::make_shared<SSDRos>();
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}

// 実行時に以下のエラーが出る場合
    // [single_shot_multibox_detector-1] (single_shot_multibox_detector:264666): Gtk-WARNING **: 22:38:02.062: Failed to parse /home/tarotsukada/.config/gtk-3.0/settings.ini: Key file does not have group “Settings”

// 編集するファイル
    // ~/.config/gtk-3.0/settings.ini

// 追記内容
    // [Settings]
    // gtk-theme-name = Adwaita
    // gtk-icon-theme-name = Adwaita
    // gtk-font-name = Sans 10
