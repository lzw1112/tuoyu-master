#include <ros/ros.h>      
#include <image_transport/image_transport.h>      
#include <cv_bridge/cv_bridge.h>      
#include <sensor_msgs/image_encodings.h>      
#include <opencv2/opencv.hpp>    
#include "autoware_config_msgs/ConfigCanny.h"
  
image_transport::Publisher pub; // 定义全局变量
int min_thresh_ = 50; // 参数 1
int max_thresh_ = 150; // 参数 2
  
void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::ImageTransport& it) {      
    // 将图像消息转换为OpenCV格式      
  
    cv_bridge::CvImagePtr cv_ptr;      
    try {      
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);      
    } catch (cv_bridge::Exception& e) {      
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());      
        return;      
    }      
    cv::Mat img = cv_ptr->image;      
      
    // 进行Canny边缘检测      
    cv::Mat edges;      
    cv::Canny(img, edges, min_thresh_, max_thresh_, 3, false);   
  
    // 将结果转换为三通道的BGR图像    
    cv::Mat edges_3ch;    
    cv::cvtColor(edges, edges_3ch, cv::COLOR_GRAY2BGR);       
      
    // 将结果转换为ROS图像消息并发布到另一个话题中      
    sensor_msgs::ImagePtr msg_edges = cv_bridge::CvImage(std_msgs::Header(), "bgr8", edges_3ch).toImageMsg();  
    // 获取图像的宽度和高度    
    int width = msg_edges->width;    
    int height = msg_edges->height;  
    std::string encoding = msg_edges->encoding;      
  
    pub.publish(msg_edges); // 使用全局变量pub     
}      
      

void update_config_params(const autoware_config_msgs::ConfigCanny::ConstPtr& param)
{
    min_thresh_ = param->min_thresh;
    max_thresh_ = param->max_thresh;
}

int main(int argc, char** argv) {      
    // 初始化ROS节点      
    ros::init(argc, argv, "canny_node");      
    ros::NodeHandle nh;      
      
    // 创建一个图像传输对象      
    image_transport::ImageTransport it(nh);  
  
    pub = it.advertise("camera/canny_image", 1); // 初始化全局变量pub        
      
    // 创建一个订阅者，订阅图像话题，并指定回调函数      
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, boost::bind(&imageCallback, _1, boost::ref(it)));      
    ros::Subscriber config_sub_ = nh.subscribe("/config/canny", 1, update_config_params);
    // 循环等待回调函数处理图像消息      
    ros::spin();      
      
    return 0;      
}
