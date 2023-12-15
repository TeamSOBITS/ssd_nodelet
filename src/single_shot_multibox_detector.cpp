#include <ssd_nodelet/single_shot_multibox_detector.hpp>

using namespace ssd_nodelet;

SingleShotMultiboxDetector::SingleShotMultiboxDetector( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path ) {
    initDNN( model_configuration_path, model_binary_path, class_names_file_path );
    setDNNParametr( 0.007843, 0.5 );
    setImgShowFlag( true );
    setUseTF( true, "base_footprint" );
    specifyDetectionObject( false, "None" );
    counter_ = 0;
}

int SingleShotMultiboxDetector::conpute(    cv::Mat& input_img,
                                            const std_msgs::Header& header,
                                            ssd_nodelet::StringArrayPtr detect_object_name,
                                            ssd_nodelet::BoundingBoxesPtr object_bbox_array,
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

    int object_num = 0;
    int detect_num = detection_mat.rows;
    double confidence_threshold = confidence_threshold_;
    bool object_specified_enabled = object_specified_enabled_;
    std::string specified_object_name = specified_object_name_;

    for (int i = 0; i < detect_num; ++i ) {
        float confidence = detection_mat.ptr<float>(i)[2];
        if ( confidence <= confidence_threshold ) continue;
        object_num++;

        size_t object_class = (size_t)(detection_mat.ptr<float>(i)[1]);
        if ( object_specified_enabled && class_names_[object_class] != specified_object_name ) continue;

        int x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * input_img.cols);
        int y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * input_img.rows);
        int x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * input_img.cols);
        int y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * input_img.rows);
        cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(x_right_top-x_left_bottom), (int)(y_right_top-y_left_bottom));

        ssd_nodelet::BoundingBox obj_bbox;
        obj_bbox.xmin = object_area.x;
        obj_bbox.ymin = object_area.y;
        obj_bbox.xmax = object_area.x + object_area.width;
        obj_bbox.ymax = object_area.y + object_area.height;
        obj_bbox.probability = confidence;
        obj_bbox.Class = class_names_[object_class];
        object_bbox_array->bounding_boxes.emplace_back(obj_bbox);
        detect_object_name->data.emplace_back(class_names_[object_class]);

        cv::rectangle(input_img, object_area, cv::Scalar(0, 255, 0) ,2);
        cv::String label = class_names_[object_class] + ": " + std::to_string(confidence);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
        cv::rectangle(input_img, label_rect, cv::Scalar::all(255), cv::FILLED);
        cv::putText(input_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));
    }
    object_bbox_array->header = header;
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header curt_header;
    curt_header.seq = counter_;
    curt_header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(curt_header, sensor_msgs::image_encodings::BGR8, input_img);
    img_bridge.toImageMsg(*result_img_msg);

    if( img_show_flag_ ){
        cv::imshow("SSD_Object_Detection Result", input_img);
        cv::waitKey(1);
    }
    return object_num;
}

int SingleShotMultiboxDetector::conpute(
    cv::Mat& input_img,
    const PointCloud::Ptr input_cloud,
    const std_msgs::Header& header,
    ssd_nodelet::PoseResult* result ) {
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

    int object_num = 0;
    int detect_num = detection_mat.rows;
    int width = input_img.cols;
    double confidence_threshold = confidence_threshold_;
    bool use_tf = use_tf_;
    std::string target_frame = target_frame_;
    bool object_specified_enabled = object_specified_enabled_;
    std::string specified_object_name = specified_object_name_;

    for (int i = 0; i < detect_num; ++i ) {
        float confidence = detection_mat.ptr<float>(i)[2];
        if ( confidence <= confidence_threshold ) continue;
        object_num++;

        size_t object_class = (size_t)(detection_mat.ptr<float>(i)[1]);
        if ( object_specified_enabled && class_names_[object_class] != specified_object_name ) continue;

        int x_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[3] * input_img.cols);
        int y_left_bottom = static_cast<int>(detection_mat.ptr<float>(i)[4] * input_img.rows);
        int x_right_top = static_cast<int>(detection_mat.ptr<float>(i)[5] * input_img.cols);
        int y_right_top = static_cast<int>(detection_mat.ptr<float>(i)[6] * input_img.rows);
        cv::Rect object_area((int)x_left_bottom , (int)y_left_bottom, (int)(x_right_top-x_left_bottom), (int)(y_right_top-y_left_bottom));

        ssd_nodelet::BoundingBox obj_bbox;
        obj_bbox.xmin = object_area.x;
        obj_bbox.ymin = object_area.y;
        obj_bbox.xmax = object_area.x + object_area.width;
        obj_bbox.ymax = object_area.y + object_area.height;
        obj_bbox.probability = confidence;
        obj_bbox.Class = class_names_[object_class];
        result->object_bbox_array->bounding_boxes.emplace_back(obj_bbox);
        result->detect_object_name->data.emplace_back(class_names_[object_class]);

        cv::rectangle(input_img, object_area, cv::Scalar(0, 255, 0) ,2);
        cv::String label = class_names_[object_class] + ": " + std::to_string(confidence);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::Rect label_rect = cv::Rect(cv::Point(object_area.x, object_area.y-label_size.height), cv::Size(label_size.width, label_size.height));
        cv::rectangle(input_img, label_rect, cv::Scalar::all(255), cv::FILLED);
        cv::putText(input_img, label, cv::Point(object_area.x, object_area.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));

        ssd_nodelet::ObjectPose obj_pose;
        obj_pose.Class = obj_bbox.Class;
        int x_ctr = ( obj_bbox.xmin + obj_bbox.xmax ) / 2;
        int y_ctr = ( obj_bbox.ymin + obj_bbox.ymax ) / 2;
        int array_num = ( width * y_ctr ) + x_ctr;

        if(std::isnan( input_cloud->points[ array_num ].x ) || std::isnan( input_cloud->points[ array_num ].y ) || std::isnan( input_cloud->points[ array_num ].z )){
            int x_ctr_min = ( x_ctr + obj_bbox.xmin ) / 2;
            int x_ctr_max = ( x_ctr + obj_bbox.xmax ) / 2;
            int y_ctr_min = ( y_ctr + obj_bbox.ymin ) / 2;
            int y_ctr_max = ( y_ctr + obj_bbox.ymax ) / 2;

            int array_num_y_ctr_min = ( width * y_ctr_min ) + x_ctr;
            int array_num_y_ctr_max = ( width * y_ctr_max ) + x_ctr;
            int array_num_x_ctr_min = ( width * y_ctr ) + x_ctr_min;
            int array_num_x_ctr_max = ( width * y_ctr ) + x_ctr_max;

            int num_pt = 0;
            PointT pt;
            if( !(std::isnan(input_cloud->points[ array_num_y_ctr_min ].x )||std::isnan(input_cloud->points[ array_num_y_ctr_min ].y)||std::isnan( input_cloud->points[array_num_y_ctr_min].z))) {
                num_pt++;
                pt.x += input_cloud->points[ array_num_y_ctr_min ].x;
                pt.y += input_cloud->points[ array_num_y_ctr_min ].y;
                pt.z += input_cloud->points[ array_num_y_ctr_min ].z;
            } if( !(std::isnan(input_cloud->points[ array_num_y_ctr_max ].x )||std::isnan(input_cloud->points[ array_num_y_ctr_max ].y)||std::isnan( input_cloud->points[array_num_y_ctr_max].z))) {
                num_pt++;
                pt.x += input_cloud->points[ array_num_y_ctr_max ].x;
                pt.y += input_cloud->points[ array_num_y_ctr_max ].y;
                pt.z += input_cloud->points[ array_num_y_ctr_max ].z;
            } if(!(std::isnan(input_cloud->points[ array_num_x_ctr_min ].x )||std::isnan(input_cloud->points[ array_num_x_ctr_min ].y)||std::isnan( input_cloud->points[array_num_x_ctr_min].z))) {
                num_pt++;
                pt.x += input_cloud->points[ array_num_x_ctr_min ].x;
                pt.y += input_cloud->points[ array_num_x_ctr_min ].y;
                pt.z += input_cloud->points[ array_num_x_ctr_min ].z;
            } if(!(std::isnan(input_cloud->points[ array_num_x_ctr_max ].x )||std::isnan(input_cloud->points[ array_num_x_ctr_max ].y)||std::isnan( input_cloud->points[array_num_x_ctr_max].z))) {
                num_pt++;
                pt.x += input_cloud->points[ array_num_x_ctr_max ].x;
                pt.y += input_cloud->points[ array_num_x_ctr_max ].y;
                pt.z += input_cloud->points[ array_num_x_ctr_max ].z;
            }
            if ( num_pt == 0 ) continue;
            obj_pose.pose.position.x = pt.x/num_pt;
            obj_pose.pose.position.y = pt.y/num_pt;
            obj_pose.pose.position.z = pt.z/num_pt;
        } else {
            obj_pose.pose.position.x = input_cloud->points[ array_num ].x;
            obj_pose.pose.position.y = input_cloud->points[ array_num ].y;
            obj_pose.pose.position.z = input_cloud->points[ array_num ].z;
        }
        obj_pose.detect_id = object_num;
        result->object_pose_array->object_poses.emplace_back( obj_pose );

        if ( !use_tf ) continue;
        br_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z)),
                ros::Time::now(),
                target_frame,
                obj_pose.Class + "_" + std::to_string(object_num)
            )
        );
    }
    result->object_bbox_array->header = header;
    result->object_pose_array->header = header;
    result->object_pose_array->header.frame_id = target_frame;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header curt_header;
    curt_header.seq = counter_;
    curt_header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(curt_header, sensor_msgs::image_encodings::BGR8, input_img);
    img_bridge.toImageMsg(*result->result_img_msg);

    if( img_show_flag_ ){
        cv::imshow("SSD_Object_Detection Result", input_img);
        cv::waitKey(23);
    }
    return object_num;
}