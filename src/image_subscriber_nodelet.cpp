#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
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

namespace ssd_nodelet {
    class ImageSubscriber : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::Subscriber sub_;

        public:
            virtual void onInit();
            void callbackImage( const sensor_msgs::ImageConstPtr& img_msg );
    };
}

void ssd_nodelet::ImageSubscriber::onInit() {
    NODELET_INFO("Listener Init");
    nh_ = getNodeHandle();
    sub_ = nh_.subscribe("pub_msg", 10, &ImageSubscriber::callbackImage, this);
}

void ssd_nodelet::ImageSubscriber::callbackImage( const sensor_msgs::ImageConstPtr& img_msg ) {

}


PLUGINLIB_EXPORT_CLASS(ssd_nodelet::ImageSubscriber, nodelet::Nodelet);
