#include <ssd_nodelet/single_shot_multibox_detector.hpp>

using namespace ssd_nodelet;

SingleShotMultiboxDetector::SingleShotMultiboxDetector( ) {
    setDNNParametr( 0.007843, 0.5 );
    setImgShowFlag( true );
    setUseTF( true, "base_footprint" );
    counter_ = 0;
}

int SingleShotMultiboxDetector::conpute(    cv::Mat& input_img, 
                                            const std_msgs::Header& header,
                                            sobit_common_msg::StringArrayPtr detect_object_name,
                                            sobit_common_msg::BoundingBoxesPtr object_bbox_array,
                                            sensor_msgs::ImagePtr result_img_msg ) {
    // Reference：https://qiita.com/wellflat/items/b32aff49846c30be8c1b

    cv::Mat image_resize;
    // Resize image to RESIZE_WIDTH x RESIZE_HEIGHT(300x300), adjust pixel values
    cv::resize(input_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
    // Converted to Blob format handled by Caffe (entity is a wrapper class for cv::Mat)
    cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
    // Input image to input layer
    net_.setInput(inputBlob, "data");
    // Forward path (forward propagation) calculation (https://iwaki2009.blogspot.com/2017/10/yolo-v2-opencvi-am-investigating-output.html)
    cv::Mat detection = net_.forward("detection_out");
    cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    sobit_common_msg::StringArray object_name;
    sobit_common_msg::BoundingBoxes bbox_array;
    int object_num = 0;
    int detect_num = detection_mat.rows;
    
    for (int i = 0; i < detect_num; i++) {
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
    std_msgs::Header curt_header;
    curt_header.seq = counter_;
    curt_header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(curt_header, sensor_msgs::image_encodings::BGR8, input_img);
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

int SingleShotMultiboxDetector::conpute(    
    cv::Mat& input_img, 
    const PointCloud::Ptr input_cloud,
    const std_msgs::Header& img_header,
    const std_msgs::Header& pc_header,
    sobit_common_msg::StringArrayPtr detect_object_name,
    sobit_common_msg::BoundingBoxesPtr object_bbox_array,
    sobit_common_msg::ObjectPoseArrayPtr object_pose_array,
    sensor_msgs::ImagePtr result_img_msg ) {
    cv::Mat image_resize;
    // Resize image to RESIZE_WIDTH x RESIZE_HEIGHT(300x300), adjust pixel values
    cv::resize(input_img, image_resize, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT));
    // Converted to Blob format handled by Caffe (entity is a wrapper class for cv::Mat)
    cv::Mat inputBlob = cv::dnn::blobFromImage(image_resize, in_scale_factor_, cv::Size(RESIZE_WIDTH, RESIZE_HEIGHT), MEAN_VAL, false);
    // Input image to input layer
    net_.setInput(inputBlob, "data");
    // Forward path (forward propagation) calculation (https://iwaki2009.blogspot.com/2017/10/yolo-v2-opencvi-am-investigating-output.html)
    cv::Mat detection = net_.forward("detection_out");
    cv::Mat detection_mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    sobit_common_msg::StringArray object_name;
    sobit_common_msg::BoundingBoxes bbox_array;
    sobit_common_msg::ObjectPoseArray obj_poses;
    int object_num = 0;
    int detect_num = detection_mat.rows;
    int width = input_img.cols;
    bool use_tf = use_tf_;
    std::string target_frame = target_frame_;

    for (int i = 0; i < detect_num; i++) {
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

        sobit_common_msg::ObjectPose obj_pose;
        obj_pose.Class = obj_bbox.Class;
        int x_ctr = ( obj_bbox.xmin + obj_bbox.xmax ) / 2;
        int y_ctr = ( obj_bbox.ymin + obj_bbox.ymax ) / 2;
        int array_num = ( width * y_ctr ) + x_ctr;
        
        if(std::isnan( input_cloud->points[ array_num ].x ) || std::isnan( input_cloud->points[ array_num ].y ) || std::isnan( input_cloud->points[ array_num ].z )){
            continue;
        }
        obj_pose.pose.position.x = input_cloud->points[ array_num ].x;
        obj_pose.pose.position.y = input_cloud->points[ array_num ].y;
        obj_pose.pose.position.z = input_cloud->points[ array_num ].z;
        obj_pose.detect_id = object_num;
        obj_poses.object_poses.push_back( obj_pose );

        if ( !use_tf ) continue;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = target_frame;
        transformStamped.child_frame_id = obj_pose.Class + "_" + std::to_string(object_num);

        // Set the translation
        transformStamped.transform.translation.x = obj_pose.pose.position.x;
        transformStamped.transform.translation.y = obj_pose.pose.position.y;
        transformStamped.transform.translation.z = obj_pose.pose.position.z;

        // Set the rotation
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;

        // tf2_ros::TransformBroadcaster
        br_.sendTransform(transformStamped);
    }
    bbox_array.header = img_header;
    obj_poses.header = pc_header;
    obj_poses.header.frame_id = target_frame;
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;
    header.seq = counter_;
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, input_img);
    img_bridge.toImageMsg(img_msg);
    

    if( img_show_flag_ ){
        cv::imshow("SSD_Object_Detection Result", input_img);
        cv::waitKey(23);
    }
    *detect_object_name = object_name;
    *object_bbox_array = bbox_array;
    *object_pose_array = obj_poses;
    *result_img_msg = img_msg;
    return object_num;
}