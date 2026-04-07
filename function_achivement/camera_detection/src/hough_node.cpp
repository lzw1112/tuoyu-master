#include <ros/ros.h>      
#include <image_transport/image_transport.h>      
#include <cv_bridge/cv_bridge.h>      
#include <sensor_msgs/image_encodings.h>      
#include <opencv2/opencv.hpp>      
#include "autoware_config_msgs/ConfigHough.h"
image_transport::Publisher pub; // 定义全局变量
int min_thresh_ = 50; // Hough 边缘检测最小阈值
int max_thresh_ = 150; // Hough 边缘检测最大阈值
int hough_thresh_ = 150; // Hough 变换阈值参数，取值范围 [0,255]
  
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

    // 使用Hough变换检测直线  
    cv::Mat lines;  
    cv::HoughLines(edges, lines, 1, CV_PI/180, hough_thresh_, 0, 0);  

    // 将检测到的直线绘制到图像上  
    for (size_t i = 0; i < lines.rows; i++) {  
        double rho = lines.at<float>(i, 0);  
        double theta = lines.at<float>(i, 1);  
        cv::Point pt1, pt2;  
        double a = cos(theta) * rho;  
        double b = sin(theta) * rho;  
        pt1.x = cvRound(a + 1000 * (-sin(theta)));  
        pt1.y = cvRound(b + 1000 * (cos(theta)));  
        pt2.x = cvRound(a - 1000 * (-sin(theta)));  
        pt2.y = cvRound(b - 1000 * (cos(theta)));  
        cv::line(img, pt1, pt2, cv::Scalar(0, 0, 255), 2); // 绘制直线到原始图像上  
    }  
      
    // 将结果转换为ROS图像消息并发布到另一个话题中      
    sensor_msgs::ImagePtr msg_edges = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg_edges); // 使用全局变量pub     
}      
      
void update_config_params(const autoware_config_msgs::ConfigHough::ConstPtr& param)
{
    
    min_thresh_ = param->min_thresh;
    max_thresh_ = param->max_thresh;
    hough_thresh_ = param->hough_thresh;

}

int main(int argc, char** argv) {      
    // 初始化ROS节点      
    ros::init(argc, argv, "hough_node");      
    ros::NodeHandle nh;      
      
    // 创建一个图像传输对象      
    image_transport::ImageTransport it(nh);  
  
    pub = it.advertise("camera/hough_image", 1); // 初始化全局变量pub        
      
    // 创建一个订阅者，订阅图像话题，并指定回调函数      
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, boost::bind(&imageCallback, _1, boost::ref(it)));      
    ros::Subscriber config_sub_ = nh.subscribe("/config/hough", 1, update_config_params);
    // 循环等待回调函数处理图像消息      
    ros::spin();      
      
    return 0;      
}
