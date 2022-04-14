#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ssd_nodelet/single_shot_multibox_detector.hpp>

namespace ssd_nodelet {
    class ImageSubscriber : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_;
            ros::Subscriber sub_img_;
            ros::Subscriber sub_ctr_;
            ssd_nodelet::SingleShotMultiboxDetector ssd_;

            bool img_show_flag_;
            bool pub_result_flag_;
            double in_scale_factor_;
            double confidence_threshold_;
            

        public:
            virtual void onInit();
            void callbackImage( const sensor_msgs::ImageConstPtr& img_msg );
            void callbackControl( const std_msgs::Bool& msg );
    };
}

void ssd_nodelet::ImageSubscriber::onInit() {
    NODELET_INFO("Listener Init");
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    img_show_flag_ = pnh_.param<bool>("ssd_img_show_flag", true);
	bool execute_flag = pnh_.param<bool>("ssd_execute_default", true);
		
	in_scale_factor_ = pnh_.param<double>("ssd_in_scale_factor", 0.007843);
	confidence_threshold_ = pnh_.param<double>("ssd_confidence_threshold", 0.5);

    std::string model_configuration_path = ros::package::getPath("ssd_node") + "/models/" + pnh_.param<std::string>("ssd_prototxt_name", "voc_object.prototxt");
    std::string model_binary_path = ros::package::getPath("ssd_node") + "/models/" + pnh_.param<std::string>("ssd_caffemodel_name", "voc_object.caffemodel");
    std::string class_names_file_path = ros::package::getPath("ssd_node") + "/models/" + pnh_.param<std::string>("ssd_class_names_file", "voc_object_names.txt");

    pub_result_flag_ = pnh_.param<bool>( "ssd_pub_result_image", true );
    std::string sub_image_topic_name = pnh_.param<std::string>( "ssd_image_topic_name", "/camera/rgb/image_raw" );
    if ( execute_flag ) sub_img_ = nh_.subscribe( sub_image_topic_name, 10, &ImageSubscriber::callbackImage, this);
    sub_ctr_ = nh_.subscribe("/ssd_nodelet/control", 10, &ImageSubscriber::callbackControl, this);

    ssd_.initDNN( model_configuration_path, model_binary_path, class_names_file_path );

}

void ssd_nodelet::ImageSubscriber::callbackImage( const sensor_msgs::ImageConstPtr& img_msg ) {
    NODELET_INFO("callbackImage");
}
void ssd_nodelet::ImageSubscriber::callbackControl( const std_msgs::Bool& msg ) {
    NODELET_INFO("callbackControl");
    std::string sub_image_topic_name = pnh_.param<std::string>( "ssd_image_topic_name", "/camera/rgb/image_raw" );
    if ( msg.data ) {
        std::cout << "[" << ros::this_node::getName() << "] Turn on the sensor subscriber\n" << std::endl;
        sub_img_ = nh_.subscribe(sub_image_topic_name, 10, &ImageSubscriber::callbackImage, this); //オン（再定義）
    } else {
        std::cout << "[" << ros::this_node::getName() << "] Turn off the sensor subscriber\n" << std::endl;
        sub_img_.shutdown();//オフ
    }
}

PLUGINLIB_EXPORT_CLASS(ssd_nodelet::ImageSubscriber, nodelet::Nodelet);
