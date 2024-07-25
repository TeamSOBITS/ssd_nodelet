#include <nodelet/nodelet.h> 
#include <pluginlib/class_list_macros.h> 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sobits_msgs/RunCtrl.h>
#include "ssd_nodelet/single_shot_multibox_detector.hpp"
#include <ros/console.h>

// Type definitions
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> ImgPCSyncPolicy;

namespace ssd_nodelet {
    class ImageCloudSubscriber : public nodelet::Nodelet {
        private:
            // Node handles
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            
            // Publishers for results
            ros::Publisher pub_object_name_;
            ros::Publisher pub_object_rect_;
            ros::Publisher pub_object_pose_;
            ros::Publisher pub_result_img_;

            // Subscribers for sensor data using message_filters
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_img_;
            std::shared_ptr<message_filters::Synchronizer<ImgPCSyncPolicy>> sync_;
    

            // Service server for controlling the nodelet
            ros::ServiceServer srv_control_;

            // SSD detector instance
            std::unique_ptr<ssd_nodelet::SingleShotMultiboxDetector> ssd_;

            // CV bridge for image conversion
            cv_bridge::CvImagePtr cv_ptr_;

            // For TF transformations
            tf::TransformListener tf_listener_;

            // Flags and configuration parameters
            bool pub_result_flag_;
            bool execute_flag_;
            int cloud_width_;
            std::string target_frame_;

            // Storage for the latest image and point cloud
            cv::Mat img_raw_;
            PointCloud::Ptr cloud_;

        public:
            virtual void onInit(); 
            bool callbackControl(sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res); 
            void callbackSenserData(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::ImageConstPtr &img_msg);
    };
}

void ssd_nodelet::ImageCloudSubscriber::onInit() {
    // Initialize node handles
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    // Load SSD model paths from parameters
    std::string model_configuration_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_prototxt_name", "voc_object.prototxt");
    std::string model_binary_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_caffemodel_name", "voc_object.caffemodel");
    std::string class_names_file_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_class_names_file", "voc_object_names.txt");
    target_frame_ = pnh_.param<std::string>("target_frame", "base_footprint");

    // Initialize SSD detector with parameters
    ssd_.reset(new ssd_nodelet::SingleShotMultiboxDetector(model_configuration_path, model_binary_path, class_names_file_path));
    ssd_->setDNNParametr(pnh_.param<double>("ssd_in_scale_factor", 0.007843), pnh_.param<double>("ssd_confidence_threshold", 0.5));
    ssd_->setImgShowFlag(pnh_.param<bool>("ssd_img_show_flag", true));
    ssd_->setUseTF(pnh_.param<bool>("use_tf", true), target_frame_);
    ssd_->specifyDetectionObject(pnh_.param<bool>("object_specified_enabled", false), pnh_.param<std::string>("specified_object_name", "None"));

    // Load configuration flags and topics from parameters
    pub_result_flag_ = pnh_.param<bool>("ssd_pub_result_image", true);
    execute_flag_ = pnh_.param<bool>("ssd_execute_default", true);
    std::string sub_image_topic_name = pnh_.param<std::string>("ssd_image_topic_name", "/camera/rgb/image_raw");
    std::string sub_cloud_topic_name = pnh_.param<std::string>("ssd_cloud_topic_name", "/camera/depth/points");

    // message_filters 
    sub_cloud_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, sub_cloud_topic_name, 1));
    sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, sub_image_topic_name, 1));
    sync_.reset(new message_filters::Synchronizer<ImgPCSyncPolicy>(ImgPCSyncPolicy(10), *sub_cloud_, *sub_img_));
    sync_->registerCallback(boost::bind(&ImageCloudSubscriber::callbackSenserData, this, _1, _2));
    
    // Advertise service server
    srv_control_ = nh_.advertiseService("detect_ctrl", &ImageCloudSubscriber::callbackControl, this);

    // Initialize publishers for the detection results
    pub_object_name_ = nh_.advertise<sobits_msgs::StringArray>("object_name", 1);
    pub_object_rect_ = nh_.advertise<sobits_msgs::BoundingBoxes>("object_rect", 1);
    pub_object_pose_ = nh_.advertise<sobits_msgs::ObjectPoseArray>("object_pose", 1);
    pub_result_img_ = nh_.advertise<sensor_msgs::Image>("detect_result", 1);

    // Allocate memory for the point cloud
    cloud_.reset(new PointCloud());
}

bool ssd_nodelet::ImageCloudSubscriber::callbackControl(sobits_msgs::RunCtrl::Request &req, sobits_msgs::RunCtrl::Response &res) {
    // Service callback to control the nodelet execution
    execute_flag_ = req.request;
    res.response = execute_flag_;

    if (execute_flag_) {
        ROS_INFO("[%s] Turn on the sensor subscriber", ros::this_node::getName().c_str());
    } else {
        ROS_INFO("[%s] Turn off the sensor subscriber", ros::this_node::getName().c_str());
    }

    return true;
}

void ssd_nodelet::ImageCloudSubscriber::callbackSenserData(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::ImageConstPtr &img_msg) {
    // Callback for synchronized sensor data

    if (!execute_flag_) {
        // If global processing is stopped, return without processing
        return;
    }


    ssd_nodelet::PoseResult result;
    PointCloud cloud_src;

    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::fromROSMsg<PointT>(*cloud_msg, cloud_src);

    try {
        // Transform the point cloud to the target frame
        tf_listener_.waitForTransform(target_frame_, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame_, ros::Time(0), cloud_src, cloud_src.header.frame_id, *cloud_, tf_listener_);
        cloud_->header.frame_id = target_frame_;
    } catch (const tf::TransformException& ex) {
        NODELET_ERROR("%s", ex.what());
        return;
    }

    try {
        // Convert ROS image message to OpenCV image
        cv_ptr_ = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        img_raw_ = cv_ptr_->image.clone();
    } catch (cv_bridge::Exception& e) {
        NODELET_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (img_raw_.empty()) {
        NODELET_ERROR("SSD_Object_Detection -> input_image error");
        return;
    }

    // Run the SSD detector
    ssd_->conpute(img_raw_, cloud_, img_msg->header, &result);

    // Publish the results
    pub_object_name_.publish(result.detect_object_name);
    pub_object_rect_.publish(result.object_bbox_array);
    pub_object_pose_.publish(result.object_pose_array);
    if (pub_result_flag_) pub_result_img_.publish(result.result_img_msg);
}

PLUGINLIB_EXPORT_CLASS(ssd_nodelet::ImageCloudSubscriber, nodelet::Nodelet);
