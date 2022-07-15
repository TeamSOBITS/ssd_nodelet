#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ssd_nodelet/single_shot_multibox_detector.hpp>

namespace ssd_nodelet {
    class ImageSubscriber : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_object_name_;
            ros::Publisher pub_object_rect_;
            ros::Publisher pub_result_img_;
            ros::Subscriber sub_img_;
            ros::Subscriber sub_ctr_;
            std::unique_ptr<ssd_nodelet::SingleShotMultiboxDetector> ssd_;
            cv_bridge::CvImagePtr cv_ptr_;

            bool pub_result_flag_;

        public:
            virtual void onInit();
            void callbackControl( const std_msgs::Bool& msg );
            void callbackImage( const sensor_msgs::ImageConstPtr& img_msg );
    };
}

void ssd_nodelet::ImageSubscriber::onInit() {
    NODELET_INFO("Listener Init");
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    std::string model_configuration_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_prototxt_name", "voc_object.prototxt");
    std::string model_binary_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_caffemodel_name", "voc_object.caffemodel");
    std::string class_names_file_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_class_names_file", "voc_object_names.txt");

    ssd_.reset( new ssd_nodelet::SingleShotMultiboxDetector( model_configuration_path, model_binary_path, class_names_file_path ) );
    ssd_->setDNNParametr( pnh_.param<double>("ssd_in_scale_factor", 0.007843), pnh_.param<double>("ssd_confidence_threshold", 0.5) );
    ssd_->setImgShowFlag( pnh_.param<bool>("ssd_img_show_flag", true) );
    ssd_->specifyDetectionObject( pnh_.param<bool>("object_specified_enabled", false), pnh_.param<std::string>("specified_object_name", "None") );

    pub_result_flag_ = pnh_.param<bool>( "ssd_pub_result_image", true );
    bool execute_flag = pnh_.param<bool>("ssd_execute_default", true);
    std::string sub_image_topic_name = pnh_.param<std::string>( "ssd_image_topic_name", "/camera/rgb/image_raw" );
    if ( execute_flag ) sub_img_ = nh_.subscribe( sub_image_topic_name, 10, &ImageSubscriber::callbackImage, this);
    sub_ctr_ = nh_.subscribe("detect_ctrl", 10, &ImageSubscriber::callbackControl, this);

    pub_object_name_  = nh_.advertise<sobit_common_msg::StringArray> ("objects_name", 1);
    pub_object_rect_ = nh_.advertise<sobit_common_msg::BoundingBoxes> ("objects_rect", 1);
    pub_result_img_ = nh_.advertise<sensor_msgs::Image>("detect_result", 1);
}

void ssd_nodelet::ImageSubscriber::callbackControl( const std_msgs::Bool& msg ) {
    // NODELET_INFO("callbackControl");
    std::string sub_image_topic_name = pnh_.param<std::string>( "ssd_image_topic_name", "/camera/rgb/image_raw" );
    if ( msg.data ) {
        std::cout << "[" << ros::this_node::getName() << "] Turn on the sensor subscriber\n" << std::endl;
        sub_img_ = nh_.subscribe(sub_image_topic_name, 10, &ImageSubscriber::callbackImage, this); //オン（再定義）
    } else {
        std::cout << "[" << ros::this_node::getName() << "] Turn off the sensor subscriber\n" << std::endl;
        sub_img_.shutdown();//オフ
    }
    return;
}

void ssd_nodelet::ImageSubscriber::callbackImage( const sensor_msgs::ImageConstPtr& img_msg ) {
    // NODELET_INFO("callbackImage");
    cv::Mat img_raw;
    sobit_common_msg::StringArrayPtr detect_object_name(new sobit_common_msg::StringArray);
    sobit_common_msg::BoundingBoxesPtr object_bbox_array(new sobit_common_msg::BoundingBoxes);
    sensor_msgs::ImagePtr result_img_msg(new sensor_msgs::Image);
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

    ssd_->conpute( img_raw, img_msg->header, detect_object_name, object_bbox_array, result_img_msg );
    pub_object_name_.publish(detect_object_name);
    pub_object_rect_.publish(object_bbox_array);
    if ( pub_result_flag_ ) pub_result_img_.publish( result_img_msg );
    return;
}

PLUGINLIB_EXPORT_CLASS(ssd_nodelet::ImageSubscriber, nodelet::Nodelet);
