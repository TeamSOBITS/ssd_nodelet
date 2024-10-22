#ifndef SINGLE_SHOT_MULTIBOX_DETECTOR_HPP
#define SINGLE_SHOT_MULTIBOX_DETECTOR_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>
//for image input
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
//for ssd processing
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
// for pcl
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
// for msg pub (original msg)
#include "sobits_msgs/StringArray.h"
#include "sobits_msgs/BoundingBox.h"
#include "sobits_msgs/BoundingBoxes.h"
#include "sobits_msgs/ObjectPoseArray.h"

constexpr size_t RESIZE_WIDTH = 300;
constexpr size_t RESIZE_HEIGHT = 300;
constexpr float  MEAN_VAL = 127.5;//Half of the maximum brightness value

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace ssd_nodelet {
    class PoseResult {
        public :
            sobits_msgs::StringArrayPtr detect_object_name;
            sobits_msgs::BoundingBoxesPtr object_bbox_array;
            sobits_msgs::ObjectPoseArrayPtr object_pose_array;
            sensor_msgs::ImagePtr result_img_msg;
            PoseResult() {
                detect_object_name.reset( new sobits_msgs::StringArray );
                object_bbox_array.reset( new sobits_msgs::BoundingBoxes );
                object_pose_array.reset( new sobits_msgs::ObjectPoseArray );
                result_img_msg.reset( new sensor_msgs::Image );
            }
    };


    class SingleShotMultiboxDetector {
        private :
            cv::dnn::Net net_;
            std::vector<std::string> class_names_;
            tf2_ros::TransformBroadcaster br_;

            double in_scale_factor_;
            double confidence_threshold_;
            bool img_show_flag_;
            bool use_tf_;
            std::string target_frame_;
            int counter_;
            bool object_specified_enabled_;
            std::string specified_object_name_;

            void initDNN( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path );
            int readFiles( const std::string& file_name, std::vector<std::string>* str_vec );

        public :
            SingleShotMultiboxDetector( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path );
            void setDNNParametr( const double in_scale_factor, const double confidence_threshold );
            void setUseTF( const bool use_tf, const std::string& target_frame );
            void setImgShowFlag( const bool img_show_flag );
            void specifyDetectionObject( const bool object_specified_enabled, const std::string& specified_object_name );

            int conpute(    cv::Mat& input_img,
                            const std_msgs::Header& header,
                            sobits_msgs::StringArrayPtr detect_object_name,
                            sobits_msgs::BoundingBoxesPtr object_bbox_array,
                            sensor_msgs::ImagePtr result_img_msg );

            int conpute(    cv::Mat& input_img,
                            const PointCloud::Ptr input_cloud,
                            const std_msgs::Header& header,
                            ssd_nodelet::PoseResult* result );
    };
}

inline void ssd_nodelet::SingleShotMultiboxDetector::initDNN( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path ) {
    // Loading a model from a configuration file
    // About the configuration file (Caffe)（https://qiita.com/Hiroki11x/items/7017ac0c03df8011b53c）
    net_ = cv::dnn::readNetFromCaffe( model_configuration_path, model_binary_path );
    // Loading a list of objects to be detected
	readFiles( class_names_file_path, &class_names_ );
}

inline int ssd_nodelet::SingleShotMultiboxDetector::readFiles( const std::string& file_name, std::vector<std::string>* str_vec ) {
    std::ifstream ifs( file_name.c_str() );
    std::string str;
    std::vector<std::string> tmp_str_vec;
    if (ifs.fail()) {
        ROS_ERROR("SSD_Object_Detection -> Read File Error");
        return -1;
    }
    while (getline(ifs, str)) tmp_str_vec.push_back(str);
    ROS_INFO_STREAM("SSD_Object_Detection -> className size = " <<  tmp_str_vec.size());
    *str_vec = tmp_str_vec;
    return 0;
}

inline void ssd_nodelet::SingleShotMultiboxDetector::setDNNParametr( const double in_scale_factor, const double confidence_threshold ) {
    in_scale_factor_ = in_scale_factor;
    confidence_threshold_ = confidence_threshold;
}

inline void ssd_nodelet::SingleShotMultiboxDetector::setImgShowFlag( const bool img_show_flag ) {
    img_show_flag_ = img_show_flag;
}

inline void ssd_nodelet::SingleShotMultiboxDetector::setUseTF( const bool use_tf, const std::string& target_frame ) {
    use_tf_ = use_tf;
    target_frame_ = target_frame;
}

inline void ssd_nodelet::SingleShotMultiboxDetector::specifyDetectionObject( const bool object_specified_enabled, const std::string& specified_object_name ) {
    object_specified_enabled_ = false;
    for ( const auto& name :class_names_ ) {
        if ( specified_object_name == name ) {
            specified_object_name_ = specified_object_name;
            object_specified_enabled_ = object_specified_enabled;
            ROS_INFO_STREAM("SSD_Object_Detection -> specified_object_name = " <<  specified_object_name.c_str() );
        }
    }
    return;
}

#endif