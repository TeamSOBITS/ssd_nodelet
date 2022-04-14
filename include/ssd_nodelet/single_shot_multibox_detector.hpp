#ifndef SINGLE_SHOT_MULTIBOX_DETECTOR_HPP
#define SINGLE_SHOT_MULTIBOX_DETECTOR_HPP

#include <ros/ros.h>
#include <ros/package.h>
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
// for msg pub
#include <sobit_common_msg/StringArray.h>
#include <sobit_common_msg/BoundingBox.h>
#include <sobit_common_msg/BoundingBoxes.h>

namespace ssd_nodelet {
    class SingleShotMultiboxDetector {
        private :
            cv::dnn::Net net_;
            std::vector<std::string> class_names_;

        public :
            SingleShotMultiboxDetector( );
            void initDNN( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path );
            int readFiles( const std::string& file_name, std::vector<std::string>* str_vec );
            void conpute( const sensor_msgs::ImageConstPtr& img );
    };
}

inline void ssd_nodelet::SingleShotMultiboxDetector::initDNN( const std::string& model_configuration_path, const std::string& model_binary_path, const std::string& class_names_file_path ) {
    net_ = cv::dnn::readNetFromCaffe( model_configuration_path, model_binary_path );
	readFiles( class_names_file_path, &class_names_ );
}

#endif