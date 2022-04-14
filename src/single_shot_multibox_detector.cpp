#include <ssd_nodelet/single_shot_multibox_detector.hpp>

using namespace ssd_nodelet;

SingleShotMultiboxDetector::SingleShotMultiboxDetector( ) {

}
int SingleShotMultiboxDetector::readFiles( const std::string& file_name, std::vector<std::string>* str_vec ) {
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
void SingleShotMultiboxDetector::conpute( const sensor_msgs::ImageConstPtr& img ) {

}