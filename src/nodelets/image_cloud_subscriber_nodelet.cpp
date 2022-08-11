#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ssd_nodelet/single_shot_multibox_detector.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> ImgPCSyncPolicy;

namespace ssd_nodelet {
    class ImageCloudSubscriber : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_object_name_;
            ros::Publisher pub_object_rect_;
            ros::Publisher pub_object_pose_;
            ros::Publisher pub_result_img_;

            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_img_;
            std::shared_ptr<message_filters::Synchronizer<ImgPCSyncPolicy>> sync_;

            ros::Subscriber sub_ctr_;
            std::unique_ptr<ssd_nodelet::SingleShotMultiboxDetector> ssd_;
            cv_bridge::CvImagePtr cv_ptr_;
            tf::TransformListener tf_listener_;

            bool pub_result_flag_;
            bool execute_flag_;
            int cloud_width_;
            std::string target_frame_;

        public:
            virtual void onInit();
            void callbackControl( const std_msgs::Bool& msg );
            void callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::ImageConstPtr &img_msg );
    };
}

void ssd_nodelet::ImageCloudSubscriber::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    std::string model_configuration_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_prototxt_name", "voc_object.prototxt");
    std::string model_binary_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_caffemodel_name", "voc_object.caffemodel");
    std::string class_names_file_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_class_names_file", "voc_object_names.txt");
    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

    ssd_.reset( new ssd_nodelet::SingleShotMultiboxDetector( model_configuration_path, model_binary_path, class_names_file_path ) );
    ssd_->setDNNParametr( pnh_.param<double>("ssd_in_scale_factor", 0.007843), pnh_.param<double>("ssd_confidence_threshold", 0.5) );
    ssd_->setImgShowFlag( pnh_.param<bool>("ssd_img_show_flag", true) );
    ssd_->setUseTF( pnh_.param<bool>("use_tf", true), target_frame_ );
    ssd_->specifyDetectionObject( pnh_.param<bool>("object_specified_enabled", false), pnh_.param<std::string>("specified_object_name", "None") );

    pub_result_flag_ = pnh_.param<bool>( "ssd_pub_result_image", true );
    execute_flag_ = pnh_.param<bool>("ssd_execute_default", true);
    std::string sub_image_topic_name = pnh_.param<std::string>( "ssd_image_topic_name", "/camera/rgb/image_raw" );
    std::string sub_cloud_topic_name = pnh_.param<std::string>( "ssd_cloud_topic_name", "/camera/depth/points" );

    // message_filters :
    sub_cloud_ .reset ( new message_filters::Subscriber<sensor_msgs::PointCloud2> ( nh_, sub_cloud_topic_name, 1 ) );
    sub_img_ .reset ( new message_filters::Subscriber<sensor_msgs::Image> ( nh_, sub_image_topic_name, 1 ) );
    sync_ .reset ( new message_filters::Synchronizer<ImgPCSyncPolicy> ( ImgPCSyncPolicy(10), *sub_cloud_, *sub_img_ ) );
    sync_ ->registerCallback ( boost::bind( &ImageCloudSubscriber::callbackSenserData, this, _1, _2 ) );
    sub_ctr_ = nh_.subscribe("detect_ctrl", 10, &ImageCloudSubscriber::callbackControl, this);

    pub_object_name_  = nh_.advertise<sobit_common_msg::StringArray> ("objects_name", 1);
    pub_object_rect_ = nh_.advertise<sobit_common_msg::BoundingBoxes> ("objects_rect", 1);
    pub_object_pose_ = nh_.advertise<sobit_common_msg::ObjectPoseArray> ("object_pose", 1);
    pub_result_img_ = nh_.advertise<sensor_msgs::Image>("detect_result", 1);
}

void ssd_nodelet::ImageCloudSubscriber::callbackControl( const std_msgs::Bool& msg ) {
    // NODELET_INFO("callbackControl");
    if ( msg.data ) std::cout << "[" << ros::this_node::getName() << "] Turn on the sensor subscriber\n" << std::endl;//オン
    else std::cout << "[" << ros::this_node::getName() << "] Turn off the sensor subscriber\n" << std::endl;//オフ
    execute_flag_ = msg.data;
    return;
}

void ssd_nodelet::ImageCloudSubscriber::callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::ImageConstPtr &img_msg ) {
    if(execute_flag_ == false){	return;	}

    cv::Mat img_raw;
    sobit_common_msg::StringArrayPtr detect_object_name(new sobit_common_msg::StringArray);
    sobit_common_msg::BoundingBoxesPtr object_bbox_array(new sobit_common_msg::BoundingBoxes);
    sobit_common_msg::ObjectPoseArrayPtr object_pose_array(new sobit_common_msg::ObjectPoseArray);
    sensor_msgs::ImagePtr result_img_msg(new sensor_msgs::Image);
    PointCloud::Ptr cloud (new PointCloud());
    std::string target_frame = target_frame_;
    PointCloud cloud_src;

    pcl::fromROSMsg<PointT>( *cloud_msg, cloud_src );
    if (target_frame.empty() == false ){
        try {
            tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *cloud, tf_listener_);
            cloud->header.frame_id = target_frame;
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
    } else ROS_ERROR("Please set the target frame.");

    try {
        cv_ptr_ = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
        img_raw = cv_ptr_->image.clone();
    } catch ( cv_bridge::Exception& e ) {
        NODELET_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (img_raw.empty() == true) {
        NODELET_ERROR("SSD_Object_Detection -> input_image error");
        return;
    }

    ssd_->conpute( img_raw, cloud, img_msg->header, cloud_msg->header, detect_object_name, object_bbox_array, object_pose_array, result_img_msg );
    // ssd_->conpute( img_raw, img_msg->header, detect_object_name, object_bbox_array, result_img_msg );
    pub_object_name_.publish(detect_object_name);
    pub_object_rect_.publish(object_bbox_array);
    pub_object_pose_.publish(object_pose_array);
    if ( pub_result_flag_ ) pub_result_img_.publish( result_img_msg );
    return;
}

PLUGINLIB_EXPORT_CLASS(ssd_nodelet::ImageCloudSubscriber, nodelet::Nodelet);
