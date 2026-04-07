/**
 * @file /include/cyrobot_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cyrobot_monitor_QNODE_HPP_
#define cyrobot_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QMutex>
#include <QLabel>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <map>
#include <QLabel>
#include <QImage>
#include <QSettings>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <ros/time.h>
#include <ros/duration.h>
#include <QTimer>
#include "gps.h"
#include "dialog.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
private:
CanThread* work;
    int init_argc;
    char** init_argv;
    QTimer *timer_sense;
    ros::Publisher chatter_publisher;
    ros::Subscriber cmdVel_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber pos_sub;
    ros::Subscriber power_sub;
    ros::Publisher goal_pub;
    ros::Publisher cmd_pub;
    ros::Publisher tof_pub;
    // 相机功能调参话题发布
    ros::Publisher config_canny_pub;
    ros::Publisher config_ROI_pub;
    ros::Publisher config_hough_pub;
    ros::Publisher config_detection_pub;

    QStringListModel logging_model;
    std::string param_string;
    int param_int;
    QMutex mu_process;
    /*****/
    ros::Subscriber stereo_sub;
    ros::Subscriber gnss_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber camera_sub;
    ros::Subscriber lslidar_sub;
    ros::Subscriber arslidar_sub;
    ros::Subscriber arslidar_sub2;
    ros::Subscriber tof_sub;

    ros::Subscriber test_sub;

    ros::Time begin_time_laser;
    ros::Time begin_time_laser2;
    ros::Time begin_time_camera;
    ros::Time begin_time_rgb;
    ros::Time begin_time_gnss;
    ros::Time begin_time_imu;
    /***/

    //图像订阅
    ros::Subscriber image_sub0;
    image_transport::Subscriber image_sub1;
    image_transport::Subscriber image_sub2;
    image_transport::Subscriber image_sub3;

    image_transport::Subscriber image_sub;
    image_transport::Subscriber image_sub_rgb;
    image_transport::Subscriber image_sub_point;
    cv::Mat img;
    cv::Mat img_rgb;
    cv::Mat img_point;
    //图像format
    QString video0_format;
    QString video1_format;
    QString video2_format;
    QString video3_format;
    QString odom_topic;
    QString power_topic;
    QString pose_topic;
    QString power_max;
    QString power_min;
    QString gnss_data;

    typedef struct
    {
        double acc_x;
        double acc_y;
        double acc_z;
    }ACC_IMU;

    typedef struct
    {
        double altitude;
        double latitude;
        double longitude;
        int    status;
    }GNSS_DATA;

    typedef struct
    {
        double width;
        double height;
        double row_step;
        double point_step ;
    }LIDAR_DATA;

    typedef struct
    {
        double time;
        double width;
        double height;
        double row_step;
        double point_step ;
    }RGB_DATA;

    typedef struct
    {
        int laser_state;
        int laser2_state;
        int camera_state;
        int rgb_state;
        int gnss_state;
        int imu_state;
    }SENSE_STATE;

    SENSE_STATE sense_sta;
    ACC_IMU acc_imu;
    GNSS_DATA gnss_msg;
    LIDAR_DATA lidar_msg;
    RGB_DATA rgb_msg;


    void update_tof_data();
    QImage Mat2QImage(cv::Mat const& src);
    void slot_update_sense();
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos);
    void speedCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void powerCallback(const std_msgs::Float32& message_holder);

    void myCallback_img(const sensor_msgs::ImageConstPtr &msg);
    void myCallback_img_rgb(const sensor_msgs::ImageConstPtr &msg);
    void myCallback_img_point(const sensor_msgs::ImageConstPtr &msg);


    void imageCallback0(const sensor_msgs::CompressedImageConstPtr& msg);
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg);
    void myCallback(const std_msgs::Float64& message_holder);

    void gnsscallback(const sensor_msgs::NavSatFix &msg);
    void imucallback(const sensor_msgs::Imu& imu);
    void lslidarcallback(const sensor_msgs::PointCloud2 &msg);

    void arslidarcallback(const std_msgs::String::ConstPtr& msg);
    void stereoCallback(const sensor_msgs::ImagePtr& msg);
    void cameraCallback(const sensor_msgs::Image& msg);
    void TopicAdvertisedTip(const char *topic);
    void testCallback(const geometry_msgs::PoseStamped& msg);
public:

	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
        bool init(const std::string &master_url, const std::string &host_url);
    void move_base(char k,float speed_linear,float speed_trun);
 
    void set_goal(QString frame,double x,double y,double z,double w);
    void Sub_Image(QString topic,int frame_id);
    int get_param_init(char* param_str);
    const char* get_param_str(char* param_str);
    QMap<QString,QString> get_topic_list();
	void run();

    /*********************
	** config msgs publish
	**********************/
    void publishe_config_canny(const int max_thresh, const int min_thresh);
    void publishe_config_hough(const int max_thresh, const int min_thresh, const int hough_thresh);
    void publishe_config_ROI(const int x, const int y, const int width, const int height);
    void publishe_config_detection(const std::string data_source, const std::string detect_type, const float iou, const float confidence, const std::string cuda);


	/*********************
	** Logging
	**********************/
	enum LogLevel {
             Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
        QImage image;
        QImage image_rgb;
        QImage image_point;
Q_SIGNALS:
    void loggingCamera();//发出设置相机图片信号
    void loggingUpdated();
    void rosShutdown();
    void acc_data(double,double,double);
    void update_gnss_data(GNSS_DATA);
    void update_lidar_data(LIDAR_DATA);
    void update_rgb_data(RGB_DATA);
    void update_sense_state(SENSE_STATE sense_state);
    void speed_x(double x);
    void speed_y(double y);

    void power(float p);
    void Master_shutdown();
    void Show_image(int,QImage);
    void position(QString frame,double x,double y,double z,double w);



};

}  // namespace cyrobot_monitor

#endif /* cyrobot_monitor_QNODE_HPP_ */
