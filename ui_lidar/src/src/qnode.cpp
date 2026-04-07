/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cyrobot_monitor/qnode.hpp"
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
//    读取topic的设置
    QSettings topic_setting("topic_setting","cyrobot_monitor");
    odom_topic= topic_setting.value("topic_odom","raw_odom").toString();
    power_topic=topic_setting.value("topic_power","power").toString();
    pose_topic=topic_setting.value("topic_amcl","amcl_pose").toString();
    power_min=topic_setting.value("power_min","10").toString();
    power_max=topic_setting.value("power_max","12").toString();
    gnss_data=topic_setting.value("gps_info","12").toString();
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"cyrobot_monitor");
    if ( ! ros::master::check() )
    {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

	// Add your ros communications here.

    //创建速度话题的订阅者
    cmdVel_sub =n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub=n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub=n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
     //导航目标点发送话题
     goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
     //速度控制话题
     cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


    // 点云功能控制信息发布器
    downsample_pub = n.advertise<std_msgs::String>("config/voxel_grid_filter", 1);
    ground_pub = n.advertise<std_msgs::String>("config/ray_ground_filter", 1);
    detect_pub = n.advertise<std_msgs::String>("config/clustering", 1);

     tof_pub = n.advertise<std_msgs::String>("tof_pub", 10);

     gnss_sub =n.subscribe("/gps/fix", 200,&QNode::gnsscallback,this);

     test_sub = n.subscribe("move_base_simple/goal", 200,&QNode::testCallback,this);
     imu_sub =n.subscribe("/imu/data_raw", 200,&QNode::imucallback,this);
     lslidar_sub =n.subscribe("/lslidar_point_cloud", 200,&QNode::lslidarcallback,this);
    arslidar_sub =n.subscribe("clusters_test", 200,&QNode::arslidarcallback,this);
    arslidar_sub2 =n.subscribe("objects_test", 200,&QNode::arslidarcallback,this);
     stereo_sub =n.subscribe("/zkhy_stereo/left/color", 200,&QNode::stereoCallback,this);
     camera_sub =n.subscribe("/usb_cam/image_raw", 200, &QNode::cameraCallback,this);

     image_transport::ImageTransport it(n);
     image_sub = it.subscribe("/usb_cam/image_raw",100,&QNode::myCallback_img,this);

     image_transport::ImageTransport it_rgb(n);
     image_sub_rgb = it_rgb.subscribe("/zkhy_stereo/left/color/theora",100,&QNode::myCallback_img_rgb,this);

    begin_time_laser = ros::Time::now();
    begin_time_laser2 = ros::Time::now();
    begin_time_camera = ros::Time::now();
    begin_time_rgb = ros::Time::now();
    begin_time_gnss = ros::Time::now();
    begin_time_imu = ros::Time::now();
    timer_sense = new QTimer(this);
    connect(timer_sense, SIGNAL(timeout()), this, SLOT(slot_update_sense()));
    timer_sense->start(1000);

  work = new CanThread;
	start();
	return true;
}

//初始化的函数*********************************
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cyrobot_monitor");
    if ( ! ros::master::check() )
    {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;




    //创建速度话题的订阅者
    cmdVel_sub =n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(),200,&QNode::speedCallback,this);
    power_sub=n.subscribe(power_topic.toStdString(),1000,&QNode::powerCallback,this);
    //机器人位置话题
    pos_sub=n.subscribe(pose_topic.toStdString(),1000,&QNode::poseCallback,this);
    //导航目标点发送话题
    goal_pub=n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
    //速度控制话题
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // 点云功能控制信息发布器
    downsample_pub = n.advertise<std_msgs::String>("config/voxel_grid_filter", 1);
    ground_pub = n.advertise<std_msgs::String>("config/ray_ground_filter", 1);
    detect_pub = n.advertise<std_msgs::String>("config/clustering", 1);

    tof_pub = n.advertise<std_msgs::String>("tof_pub", 10);

    gnss_sub =n.subscribe("/gps/fix", 200,&QNode::gnsscallback,this);

    test_sub = n.subscribe("move_base_simple/goal", 200,&QNode::testCallback,this);
    imu_sub =n.subscribe("/imu/data_raw", 200,&QNode::imucallback,this);
    lslidar_sub =n.subscribe("/lslidar_point_cloud", 200,&QNode::lslidarcallback,this);
    arslidar_sub =n.subscribe("clusters_test", 200,&QNode::arslidarcallback,this);
    arslidar_sub2 =n.subscribe("objects_test", 200,&QNode::arslidarcallback,this);
    stereo_sub =n.subscribe("/zkhy_stereo/left/color", 200,&QNode::stereoCallback,this);
    //camera_sub =n.subscribe("/usb_cam/image_raw", 200, &QNode::cameraCallback,this);

    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("/usb_cam/image_raw",100,&QNode::myCallback_img,this);

    image_transport::ImageTransport it_rgb(n);
    image_sub_rgb = it_rgb.subscribe("/zkhy_stereo/left/color/theora",100,&QNode::myCallback_img_rgb,this);

  work = new CanThread;

    //image_transport::ImageTransport it_point(n);
    //image_sub_point = it_point.subscribe("/zkhy_stereo/points",100,&QNode::myCallback_img_point,this);

    timer_sense = new QTimer(this);
    timer_sense->start(1000);
    connect(timer_sense, SIGNAL(timeout()), this, SLOT(slot_update_sense()));

    begin_time_laser = ros::Time::now();
    begin_time_laser2 = ros::Time::now();
    begin_time_camera = ros::Time::now();
    begin_time_rgb = ros::Time::now();
    begin_time_gnss = ros::Time::now();
    begin_time_imu = ros::Time::now();

	start();
	return true;
}

QMap<QString,QString> QNode::get_topic_list()
{
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);
    QMap<QString,QString> res;
    for(auto topic:topic_list)
    {

        res.insert(QString::fromStdString(topic.name),QString::fromStdString(topic.datatype));

    }
    return res;
}

void QNode::slot_update_sense()
{
   
}

//机器人位置话题的回调函数
void QNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pos)
{
    emit position(pos.header.frame_id.data(), pos.pose.pose.position.x,pos.pose.pose.position.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
//    qDebug()<<<<" "<<pos.pose.pose.position.y;
}
void QNode::powerCallback(const std_msgs::Float32 &message_holder)
{

    emit power(message_holder.data);
}
void QNode::testCallback(const geometry_msgs::PoseStamped& msg)
{
    ROS_INFO_STREAM(msg);
}

void QNode::gnsscallback(const sensor_msgs::NavSatFix &msg)
{
    begin_time_gnss = ros::Time::now();

    gnss_msg.altitude = msg.altitude;
    gnss_msg.latitude = msg.latitude;
    gnss_msg.longitude = msg.longitude;
    gnss_msg.status = msg.status.status;
    emit update_gnss_data(gnss_msg);
}
void QNode::imucallback(const sensor_msgs::Imu& imu)
{
    begin_time_imu = ros::Time::now();

    acc_imu.acc_x = imu.angular_velocity.x;
    acc_imu.acc_y = imu.angular_velocity.y;
    acc_imu.acc_z = imu.angular_velocity.z;
    emit acc_data(acc_imu.acc_x,acc_imu.acc_y,acc_imu.acc_z);
}
void QNode::lslidarcallback(const sensor_msgs::PointCloud2 &msg)
{
    begin_time_laser = ros::Time::now();

    lidar_msg.width = msg.width;
    lidar_msg.height = msg.height;
    lidar_msg.row_step = msg.row_step;
    lidar_msg.point_step = msg.point_step;
    emit update_lidar_data(lidar_msg);
}
void QNode::arslidarcallback(const std_msgs::String::ConstPtr& msg)
{
    begin_time_laser2 = ros::Time::now();
}
void QNode::stereoCallback(const sensor_msgs::ImagePtr& msg)
{
   begin_time_rgb = ros::Time::now();
   //rgb_msg.time = msg.header.stamp.now().sec;
   //rgb_msg.width = msg.width;
   //rgb_msg.height = msg.height;
   //rgb_msg.row_step = msg.row_step;
   //rgb_msg.point_step = msg.point_step;

   //emit update_rgb_data(rgb_msg);
}
void QNode::cameraCallback(const sensor_msgs::Image& msg)
{
    begin_time_camera = ros::Time::now();
}

void QNode::myCallback(const std_msgs::Float64 &message_holder)
{
    qDebug()<<message_holder.data<<endl;
}
//发布导航目标点信息
void QNode::set_goal(QString frame,double x,double y,double z,double w)
{
    geometry_msgs::PoseStamped goal;
    //设置frame
    goal.header.frame_id=frame.toStdString();
    //设置时刻
    goal.header.stamp=ros::Time::now();
    goal.pose.position.x=x;
    goal.pose.position.y=y;
    goal.pose.position.z=0;
    goal.pose.orientation.z=z;
    goal.pose.orientation.w=w;
    goal_pub.publish(goal);
    ros::spinOnce();
}
//速度回调函数
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    emit speed_x(msg->twist.twist.linear.x);
    emit speed_y(msg->twist.twist.linear.y);
}
void QNode::run() {
        int count=0;
        ros::Rate loop_rate(40);
        //当当前节点没有关闭时
        while ( ros::ok() ) 
        {
            //调用消息处理回调函数
            ros::spinOnce();
            update_tof_data();
            if((ros::Time::now() - begin_time_laser) < ros::Duration(0.5))
            {
                sense_sta.laser_state = 1;
            }
            else
            {
                sense_sta.laser_state = 0;
            }
            if((ros::Time::now() - begin_time_laser2) < ros::Duration(0.5))
            {
                sense_sta.laser2_state  = 1;
            }
            else
            {
                sense_sta.laser2_state  = 0;
            }
            if((ros::Time::now() - begin_time_camera) < ros::Duration(0.5))
            {
                sense_sta.camera_state  = 1;
            }
            else
            {
               sense_sta.camera_state  = 0;
            }
             if((ros::Time::now() - begin_time_rgb) < ros::Duration(0.5))
            {
                sense_sta.rgb_state  = 1;
            }
            else
            {
                sense_sta.rgb_state  = 0;
            }   
            if((ros::Time::now() - begin_time_gnss) < ros::Duration(0.5))
            {
                sense_sta.gnss_state  = 1;
            }
            else
            {
                sense_sta.gnss_state  = 0;
            }
            if((ros::Time::now() - begin_time_imu) <  ros::Duration(0.5))
            {
                sense_sta.imu_state  = 1;
            }
            else
            {
                sense_sta.imu_state  = 0;
            }

            emit update_sense_state(sense_sta);
            loop_rate.sleep();
        }
        //如果当前节点关闭
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)


}

void QNode::update_tof_data()
{
		std_msgs::String msg;
      std::string str = std::string("tof_l_dis :")+
                        std::to_string(work->tof_data().tof_l_dis) +
                         std::string("  ----- ")+
                         std::string("tof_r_dis: ")+
                         std::to_string(work->tof_data().tof_r_dis);
        msg.data = str;
        tof_pub.publish(msg);
		ros::spinOnce();
}


//发布机器人速度控制
 void QNode::move_base(char k,float speed_linear,float speed_trun)
 {
     char yaml_set[200];
     sprintf(yaml_set, "/move_base/global_costmap/height");
     ROS_INFO("get int_param move_base: %d", get_param_init(yaml_set));

     sprintf(yaml_set, "/move_base/TebLocalPlannerROS/map_frame");
     ROS_INFO("get move_base str_param: %s", get_param_str(yaml_set));

     std::map<char, std::vector<float>> moveBindings
     {
       {'i', {1, 0, 0, 0}},
       {'o', {1, 0, 0, -1}},
       {'j', {0, 0, 0, 1}},
       {'l', {0, 0, 0, -1}},
       {'u', {1, 0, 0, 1}},
       {',', {-1, 0, 0, 0}},
       {'.', {-1, 0, 0, 1}},
       {'m', {-1, 0, 0, -1}},
       {'O', {1, -1, 0, 0}},
       {'I', {1, 0, 0, 0}},
       {'J', {0, 1, 0, 0}},
       {'L', {0, -1, 0, 0}},
       {'U', {1, 1, 0, 0}},
       {'<', {-1, 0, 0, 0}},
       {'>', {-1, -1, 0, 0}},
       {'M', {-1, 1, 0, 0}},
       {'t', {0, 0, 1, 0}},
       {'b', {0, 0, -1, 0}},
       {'k', {0, 0, 0, 0}},
       {'K', {0, 0, 0, 0}}
     };
     char key=k;
     //计算是往哪个方向
     float x = moveBindings[key][0];
     float y = moveBindings[key][1];
     float z = moveBindings[key][2];
     float th = moveBindings[key][3];
     //计算线速度和角速度
     float speed = speed_linear;
     float turn = speed_trun;
     // Update the Twist message
     geometry_msgs::Twist twist;
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    cmd_pub.publish(twist);
    ros::spinOnce();

 }

int QNode::get_param_init(char* param_str)
{
    ros::NodeHandle pn("~my_namespce");
    mu_process.lock();
    pn.param<int>(param_str, param_int, 666);
    pn.getParam(param_str, param_int);
    ROS_INFO("int_param: %d", param_int);
    mu_process.unlock();

    return param_int;
}

void QNode::TopicAdvertisedTip(const char *topic)
{
  // ROS_INFO("%s has been advertised,use 'rostopic "
  //          "echo /%s' to view the data",
  //          topic, topic);
  std::cout << topic << "has been advertised,use rostopic echo /" << topic << " to view the data" << "\n";
}

const char*QNode::get_param_str(char* param_str)
{
    ros::NodeHandle n;

    mu_process.lock();
    n.param<std::string>(param_str, param_string, "haha");
    n.getParam(param_str, param_string);
    ROS_INFO("int_str_param: %s", param_string.c_str());

    mu_process.unlock();

    return  param_string.c_str();
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img = cv_ptr->image;
        image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format

        //Q_EMIT loggingCamera();
    }
    catch (cv_bridge::Exception& e)
    {

    }
}

void QNode::myCallback_img_rgb(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img_rgb = cv_ptr->image;
        image_rgb = QImage(img_rgb.data,img_rgb.cols,img_rgb.rows,img_rgb.step[0],QImage::Format_RGB888);//change  to QImage format

        //Q_EMIT loggingCamera();
    }
    catch (cv_bridge::Exception& e)
    {

    }
}

void QNode::myCallback_img_point(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img_point = cv_ptr->image;
        image_point = QImage(img_point.data,img_point.cols,img_point.rows,img_point.step[0],QImage::Format_RGB888);//change  to QImage format

        //Q_EMIT loggingCamera();
    }
    catch (cv_bridge::Exception& e)
    {

    }
}

 //订阅图片话题，并在label上显示
 void QNode::Sub_Image(QString topic,int frame_id)
 {
      ros::NodeHandle n;
      image_transport::ImageTransport it_(n);
     switch (frame_id) {
         case 0:
            image_sub0=n.subscribe(topic.toStdString(),100,&QNode::imageCallback0,this);
         break;
         case 1:
             image_sub1=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback1,this);
          break;
         case 2:
             image_sub2=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback2,this);
          break;
         case 3:
             image_sub3=it_.subscribe(topic.toStdString(),100,&QNode::imageCallback3,this);
          break;
     }
     ros::spinOnce();
 }

 //图像话题的回调函数
 void QNode::imageCallback0(const sensor_msgs::CompressedImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
       if(msg->format.substr(0,4)=="rgb8"){
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
       }
       else{
         cv_ptr = cv_bridge::toCvCopy(msg,msg->format.substr(0,4));
       }
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(0,im);
       }
       catch (std::runtime_error& e)
       {
         log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg,msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(1,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame1 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(2,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame2 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 //图像话题的回调函数
 void QNode::imageCallback3(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_image(3,im);
       }
       catch (cv_bridge::Exception& e)
       {
         log(Error,("video frame3 exception: "+QString(e.what())).toStdString());
         return;
       }
 }
 QImage QNode::Mat2QImage(cv::Mat const& src)
 {
   QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

   const float scale = 255.0;

   if (src.depth() == CV_8U) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = src.at<quint8>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
         }
       }
     }
   } else if (src.depth() == CV_32F) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = scale * src.at<float>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
           dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
         }
       }
     }
   }

   return dest;
 }
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace cyrobot_monitor
