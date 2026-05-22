#include "ssd_ros/single_shot_multibox_detector.hpp"

constexpr size_t RESIZE_WIDTH = 300;
constexpr size_t RESIZE_HEIGHT = 300;
constexpr float  MEAN_VAL = 127.5; //Half of the maximum brightness value


void SSDRos::read_files() {
    try {
        YAML::Node config = YAML::LoadFile(class_names_file_path);
        // Add "background" class at the beginning of the class names list
        class_names_.clear();
        class_names_.push_back("background");
        auto class_names = config["class"];

        for (const auto class_name : class_names) {
            class_names_.push_back(class_name.as<std::string>());
        }
        in_scale_factor_ = config["in_scale_factor"].as<double>();

        return;
    } catch (const YAML::Exception& e) {
        std::cout << "Failed to open the YAML file..." << std::endl;
        return;
    }
}



bool SSDRos::isPath_correct(const std::string& base_path) {
    // Reset paths
    model_configuration_path.clear();
    model_binary_path.clear();
    class_names_file_path.clear();

    // Exists and is a directory
    if (!std::filesystem::exists(base_path) || !std::filesystem::is_directory(base_path)) {
        return false;
    }

    std::vector<std::string> prototxt_files;
    std::vector<std::string> caffemodel_files;
    std::vector<std::string> yaml_files;

    // ディレクトリ走査
    for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
        // Skip if not a regular file
        if (!entry.is_regular_file()) {
            continue;
        }

        // Get file extension
        std::string ext = entry.path().extension().string();

        if (ext == ".prototxt") {
            prototxt_files.push_back(entry.path().string());
        }
        else if (ext == ".caffemodel") {
            caffemodel_files.push_back(entry.path().string());
        }
        else if (ext == ".yaml") {
            yaml_files.push_back(entry.path().string());
        }
    }

    // Check if exactly one file of each type is found
    if (prototxt_files.size() != 1 ||
        caffemodel_files.size() != 1 ||
        yaml_files.size() != 1) {
        return false;
    }

    // Store paths
    model_configuration_path = prototxt_files[0];
    model_binary_path = caffemodel_files[0];
    class_names_file_path = yaml_files[0];

    return true;
}


void SSDRos::callback_image(const std::shared_ptr<sensor_msgs::msg::Image> img_msg) {
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

        if (object_specified_enabled_) {
            bool specified_class_found = false;
            for (const auto& specified_class : specified_object_classes_) {
                if (class_names_[(size_t)(detection_mat.ptr<float>(i)[1])] == specified_class) {
                    specified_class_found = true;
                    break;
                }
            }
            if (!specified_class_found) continue;
        }

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


void SSDRos::callback_RunCtr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    rclcpp::QoS qos_profile(1); // depth = 1

    if (qos_reliability_ == "BEST_EFFORT")
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    else if (qos_reliability_ == "RELIABLE")
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    else
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    if (req->data) {
        if (!sub_image_) sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(topic_name, qos_profile, std::bind(&SSDRos::callback_image, this, std::placeholders::_1));
    } else {
        if (sub_image_)  sub_image_.reset();
    }
    res->success = true;
}


SSDRos::SSDRos(): Node("single_shot_multibox_detector_node") {
    this->declare_parameter("image_topic_name", "image_raw");
    this->declare_parameter("qos_reliability", "BEST_EFFORT");
    this->declare_parameter<std::vector<std::string>>("specified_object_class", std::vector<std::string>{});
    this->declare_parameter("model_directory", "../models/objects_model");
    this->declare_parameter("execute_default", true);
    this->declare_parameter("image_show_flag", true);
    this->declare_parameter("confidence_threshold", 0.5);

    topic_name = this->get_parameter("image_topic_name").as_string();
    qos_reliability_ = this->get_parameter("qos_reliability").as_string();
    img_show_flag_ = this->get_parameter("image_show_flag").as_bool();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
    this->get_parameter_or("specified_object_class", specified_object_classes_, std::vector<std::string>{});

    if (!specified_object_classes_.empty()) {
        if (specified_object_classes_.back() == "None" || specified_object_classes_.back() == "dummy") {
            specified_object_classes_.clear();
        }
    }

    object_specified_enabled_ = !specified_object_classes_.empty();

    if (!isPath_correct(this->get_parameter("model_directory").as_string())) {
        RCLCPP_ERROR(this->get_logger(), "SSD_Object_Detection -> Path Error");
        return;
    }

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


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nd = std::make_shared<SSDRos>();
    rclcpp::spin(nd);
    rclcpp::shutdown();
    return 0;
}
