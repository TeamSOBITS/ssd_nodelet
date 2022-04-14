#include <ssd_nodelet/single_shot_multibox_detector.hpp>

using namespace ssd_nodelet;

SingleShotMultiboxDetector::SingleShotMultiboxDetector( ) {
    setDNNParametr( 0.007843, 0.5 );
    setImgShowFlag( true );
}

int SingleShotMultiboxDetector::conpute(    cv::Mat& input_img, 
                                            const std_msgs::Header& header,
                                            sobit_common_msg::StringArrayPtr detect_object_name,
                                            sobit_common_msg::BoundingBoxesPtr object_bbox_array,
                                            sensor_msgs::ImagePtr result_img_msg ) {
    // 参照：https://qiita.com/wellflat/items/b32aff49846c30be8c1b

    cv::Mat image_resize;
    // RESIZE_WIDTH x RESIZE_HEIGHT(300x300)に画像をリサイズ、画素値を調整
    cv::resize(input_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
    // Caffeで扱うBlob形式に変換 (実体はcv::Matのラッパークラス)
    cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
    // 入力層に画像を入力
    net_.setInput(inputBlob, "data");
    // フォワードパス(順伝播)の計算(https://iwaki2009.blogspot.com/2017/10/yolo-v2-opencvi-am-investigating-output.html)
    cv::Mat detection = net_.forward("detection_out");
    cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    sobit_common_msg::StringArray object_name;
    sobit_common_msg::BoundingBoxes bbox_array;
    int object_num = 0;
    
    for (int i = 0; i < detection_mat.rows; i++) {
        float confidence = detection_mat.ptr<float>(i)[2];
        if ( confidence <= confidence_threshold_ ) continue;
        object_num++;

        size_t object_class = (size_t)(detection_mat.ptr<float>(i)[1]);

        int x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * input_img.cols);
        int y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * input_img.rows);
        int x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * input_img.cols);
        int y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * input_img.rows);
        cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(x_right_top-x_left_bottom), (int)(y_right_top-y_left_bottom));

        sobit_common_msg::BoundingBox obj_bbox;
        obj_bbox.xmin = object_area.x;
        obj_bbox.ymin = object_area.y;
        obj_bbox.xmax = object_area.x + object_area.width;
        obj_bbox.ymax = object_area.y + object_area.height;
        obj_bbox.probability = confidence;
        obj_bbox.Class = class_names_[object_class];
        bbox_array.bounding_boxes.push_back(obj_bbox);
        object_name.data.push_back(class_names_[object_class]);

        cv::rectangle(input_img, object_area, cv::Scalar(0, 255, 0) ,2);
        cv::String label = class_names_[object_class] + ": " + std::to_string(confidence);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
        cv::rectangle(input_img, label_rect, cv::Scalar::all(255), CV_FILLED);
        cv::putText(input_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));
    }
    bbox_array.header = header;
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_bridge;
    // std_msgs::Header header;
    // header.seq = this->counter; // user defined counter
    // header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, input_img);
    img_bridge.toImageMsg(img_msg);
    

    if( img_show_flag_ ){
        cv::imshow("SSD_Object_Detection Result", input_img);
        cv::waitKey(1);
    }
    *detect_object_name = object_name;
    *object_bbox_array = bbox_array;
    *result_img_msg = img_msg;
    return object_num;
}