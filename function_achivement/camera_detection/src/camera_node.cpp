#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/opencv.hpp>  
#include <cv_bridge/cv_bridge.h>  
  
int main(int argc, char** argv) {  
  ros::init(argc, argv, "opencv_camera_publisher");  
  ros::NodeHandle nh;  
  image_transport::ImageTransport it(nh);  
  image_transport::Publisher pub = it.advertise("camera/image", 1);  
  
  cv::VideoCapture cap(1);  // 使用默认相机  
  if(!cap.isOpened()) {  
    ROS_ERROR("Error opening camera!");  
    return -1;  
  }  
  
  ros::Rate rate(30);  // 发布频率30Hz  
  while(ros::ok()) {  
    cv::Mat frame;  
    cap >> frame;
    // 定义新的图像大小  
    cv::Size newSize(640, 480);  
    // 创建一个新的图像矩阵  
    cv::Mat resizedImage(newSize, frame.type());  
    // 调整图像大小  
    cv::resize(frame, resizedImage, newSize);

    if(frame.empty()) {  
      ROS_ERROR("Error reading camera!");  
      return -1;  
    }  
  
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
    pub.publish(msg);  
    rate.sleep();  
  }  
}
