/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QMouseEvent>
#include <QtGui>
#include <QPalette>
#include <QMessageBox>
#include <iostream>
#include "../include/cyrobot_monitor/main_window.hpp"
#include <QString>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <QFileDialog>
#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <fstream>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    ui.label_22->setStyleSheet("color: #00008B;");
    initUis();
    initData();
    can_cfg_flag = 0;
    time_calib = 0;
    pub_cnt = 0;
    //读取配置文件
    ReadSettings();
    setWindowIcon(QIcon(":/images/robot.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	//ui.view_logging->setModel(qnode.loggingModel());
    send_id = 0;
    gnss_cnt = 0;
    /*********************
    ** 自动连接master
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    //链接connect
    connections();

}
//订阅video话题
void MainWindow::initVideos()
{

   QSettings video_topic_setting("video_topic","cyrobot_monitor");
   QStringList names=video_topic_setting.value("names").toStringList();
   QStringList topics=video_topic_setting.value("topics").toStringList();
   if(names.size()==4)
   {

   }
   if(topics.size()==4)
   {
       if(topics[0]!="")
        qnode.Sub_Image(topics[0],0);
       if(topics[1]!="")
        qnode.Sub_Image(topics[1],1);
       if(topics[2]!="")
        qnode.Sub_Image(topics[2],2);
       if(topics[3]!="")
        qnode.Sub_Image(topics[3],3);

   }

   //链接槽函数
   connect(&qnode,SIGNAL(Show_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));
}
void MainWindow::slot_show_image(int frame_id, QImage image)
{
    switch (frame_id)
    {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    }
}

//鼠标双击界面全屏
void MainWindow::mouseDoubleClickEvent (QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton){
        if(ui.menubar->isHidden())
            ui.menubar->show();
        else ui.menubar->close();
        if(windowState() != Qt::WindowFullScreen)
        {
          setWindowState(Qt::WindowFullScreen);
        }
        else
        {
          setWindowState(Qt::WindowMaximized);
        }
    }
}

#define CMD_RESULT_BUF_SIZE 1024

/*
 * cmd：待执行命令
 * result：命令输出结果
 * 函数返回：0 成功；-1 失败；
 */
int ExecuteCMD(const char *cmd, char *result)
{
    int iRet = -1;
    char buf_ps[CMD_RESULT_BUF_SIZE];
    char ps[CMD_RESULT_BUF_SIZE] = {0};
    FILE *ptr;
    std::strcpy(ps, cmd);
    if((ptr = popen(ps, "r")) != NULL)
    {
        while(fgets(buf_ps, sizeof(buf_ps), ptr) != NULL)
        {
           strcat(result, buf_ps);
           if(strlen(result) > CMD_RESULT_BUF_SIZE)
           {
               break;
           }
        }
        pclose(ptr);
        ptr = NULL;
        iRet = 0;  // 处理成功
    }
    else
    {
        // printf("popen %s error\n", ps);
        iRet = -1; // 处理失败
    }

    return iRet;
}


//初始化UI
void MainWindow::initUis()
{
	YAML::Node config;
	char path_name[200];
//

    // char result[CMD_RESULT_BUF_SIZE]={0};
    // std::string str1 = "cat /sys/class/net/enp1s0/address"; //dc:a6:32:f7:9c:94
    // const char *command1 = str1.c_str();
    // ExecuteCMD(command1, result);
    // std::string findMac = result;
    //  if(findMac.length() > 17)
    //  {
    //         std::string macAddress = config["mac"].as<std::string>();
    //         if(strcmp(findMac.substr(0,17).c_str(),macAddress.c_str())==0){
    //         printf("==============\n");
            
    //         }
    //         else{
    //                  printf("!!!!!!!!!!!==============\n");
            
    //                 return;
    //         }
    //  }


    laser_state = 0;
    laser2_state = 0;
    camera_state = 0;
    rgb_state = 0;
    gnss_state = 0;
    imu_state = 0;

    char yaml_get[200];


    // sprintf(yaml_get,
    // "gnome-terminal rosnode kill --all");
    //  printf("-------------- %s %s ----------\n", __func__, yaml_get);
    //  system(yaml_get);

    QFont font("Microsoft YaHei", 10, 75);
    ui.label_camera_state->setFont(font);
    ui.label_binary_state->setFont(font);

    QPalette pa;
    pa.setColor(QPalette::WindowText, QColor(255,255,255));


    ui.label_camera_state->setPalette(pa);
    ui.label_camera_state->setText("离线");

    ui.label_binary_state->setPalette(pa);
    ui.label_binary_state->setText("离线");


    block = QColor(71, 71, 71);
    green = QColor(0, 255, 0);

    ui.tab_manager->setTabEnabled(1,false);

    ui.tab_manager->setCurrentIndex(0);
  
    //Global options
    QTreeWidgetItem *Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));

    QTreeWidgetItem* FixedFrame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    Global->addChild(FixedFrame);

    Global->setExpanded(true);
    //添加combox控件
    QComboBox *frame=new QComboBox();
    frame->addItem("map");
    frame->setEditable(true);
    frame->setMaximumWidth(150);

    QTreeWidgetItem* bcolor=new QTreeWidgetItem(QStringList()<<"Background Color");
    Global->addChild(bcolor);
    //添加lineedit控件
    QLineEdit *colorval=new QLineEdit("48;48;48");
    colorval->setMaximumWidth(150);

    QSpinBox *framerateval=new QSpinBox();
    framerateval->setStyleSheet("border:none");
    framerateval->setMaximumWidth(150);
    framerateval->setRange(10,50);
    framerateval->setValue(30);
    QTreeWidgetItem* framerate=new QTreeWidgetItem(QStringList()<<"Frame Rate");
    Global->addChild(framerate);

    //grid
    QTreeWidgetItem *Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));


    Grid->setExpanded(true);
    QCheckBox* gridcheck=new QCheckBox;
    gridcheck->setChecked(true);


    QTreeWidgetItem *Grid_Status=new QTreeWidgetItem(QStringList()<<"Statue:");
    Grid_Status->setIcon(0,QIcon("://images/ok.png"));
    Grid->addChild(Grid_Status);
    QLabel *Grid_Status_Value=new QLabel("ok");
    Grid_Status_Value->setMaximumWidth(150);


    QTreeWidgetItem* Reference_Frame=new QTreeWidgetItem(QStringList()<<"Reference Frame");
    QComboBox* Reference_Frame_Value=new QComboBox();
    Grid->addChild(Reference_Frame);
    Reference_Frame_Value->setMaximumWidth(150);
    Reference_Frame_Value->setEditable(true);
    Reference_Frame_Value->addItem("<Fixed Frame>");


    QTreeWidgetItem* Plan_Cell_Count=new QTreeWidgetItem(QStringList()<<"Plan Cell Count");
    Grid->addChild(Plan_Cell_Count);
    QSpinBox* Plan_Cell_Count_Value=new QSpinBox();

    Plan_Cell_Count_Value->setMaximumWidth(150);
    Plan_Cell_Count_Value->setRange(1,100);
    Plan_Cell_Count_Value->setValue(10);

    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    QLineEdit* Grid_Color_Value=new QLineEdit();
    Grid_Color_Value->setMaximumWidth(150);
    Grid->addChild(Grid_Color);

    Grid_Color_Value->setText("160;160;160");


    //qucik treewidget
    subthread =  new QThread;;
    work = new CanThread;

    work->moveToThread(subthread);

    connect(subthread, SIGNAL(finished()), subthread, SLOT(deleteLater()));        //终止线程时要调用deleteLater槽函数
    connect(subthread, SIGNAL(started()), work, SLOT(startThreadSlot()));  //开启线程槽函数
    //connect(subthread,SIGNAL(finished()),this,SLOT(finishedThreadSlot()));
    subthread->start();

    button_switch_camera = new SwitchButton;
    button_switch_rgb = new SwitchButton;
    button_switch_rgbd = new SwitchButton;
    button_switch_point = new SwitchButton;

    button_switch_ROI = new SwitchButton;
    button_switch_canny = new SwitchButton;
    button_switch_hough = new SwitchButton;
    button_switch_detection = new SwitchButton;

    button_switch_ROI->setBgColorOn(green);
    button_switch_ROI->setBgColorOff(block);

    button_switch_canny->setBgColorOn(green);
    button_switch_canny->setBgColorOff(block);

    button_switch_hough->setBgColorOn(green);
    button_switch_hough->setBgColorOff(block);

    button_switch_detection->setBgColorOn(green);
    button_switch_detection->setBgColorOff(block);

    button_switch_camera->setBgColorOn(green);
    button_switch_camera->setBgColorOff(block);

    button_switch_rgb->setBgColorOn(green);
    button_switch_rgb->setBgColorOff(block);

    button_switch_rgbd->setBgColorOn(green);
    button_switch_rgbd->setBgColorOff(block);

    button_switch_point->setBgColorOn(green);
    button_switch_point->setBgColorOff(block);


    ui.horizontalLayout_camera->insertWidget(1, button_switch_camera);
    ui.horizontalLayout_rgb->insertWidget(1, button_switch_rgb);

    ui.horizontalLayout_ROI->insertWidget(1, button_switch_ROI);
    ui.horizontalLayout_hough->insertWidget(1, button_switch_hough);
    ui.horizontalLayout_canny->insertWidget(1, button_switch_canny);
    ui.horizontalLayout_detection->insertWidget(1, button_switch_detection);


    connect(button_switch_rgb, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_stereo(bool)));
    connect(button_switch_camera, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_camera(bool)));

    connect(button_switch_canny, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_canny(bool)));
    connect(button_switch_hough, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_hough(bool)));
    connect(button_switch_ROI, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_ROI(bool)));
    connect(button_switch_detection, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_detection(bool)));

    connect(button_switch_rgbd, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_rgbd(bool)));
    connect(button_switch_point, SIGNAL(checkedChanged(bool)),this,SLOT(slot_startUp_point(bool)));


    //connect(timer_tof, SIGNAL(timeout()), this, SLOT(slot_startUp_tof()));
    //timer_tof->start(1000);//start之后,每隔一秒触发一次槽函数
}
void MainWindow::initRviz()
{

}

void MainWindow::RvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;

}

///
/// \brief 检查重名
/// \param name
/// \return
///
QString MainWindow::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                //m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else
                    {
                        name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}

void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);
}

void MainWindow::connections()
{
    timer_tof = new QTimer(this);
    timer_tof->start(500);
    connect(timer_tof, SIGNAL(timeout()), this, SLOT(slot_update_tof()));

    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));

    //connect速度的信号
    connect(&qnode,SIGNAL(speed_x(double)),this,SLOT(slot_speed_x(double)));
    connect(&qnode,SIGNAL(speed_y(double)),this,SLOT(slot_speed_y(double)));
    //电源的信号
    connect(&qnode,SIGNAL(power(float)),this,SLOT(slot_power(float)));
    //机器人位置信号
    connect(&qnode,SIGNAL(position(QString,double,double,double,double)),this,SLOT(slot_position_change(QString,double,double,double,double)));

    connect(work, SIGNAL(sen_state(Radar_STATE)),this,SLOT(update_sensor_data(Radar_STATE)));
    qRegisterMetaType<SENSE_STATE>("SENSE_STATE");
    connect(&qnode,SIGNAL(update_sense_state(SENSE_STATE)),this,
                SLOT(slot_update_ros_sense_data(SENSE_STATE)));

    QObject::connect(&qnode,SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));
    //QObject::connect(&qnode,SIGNAL(loggingCamera_rgb()),this,SLOT(updateLogcamera_rgb()));
    //QObject::connect(&qnode,SIGNAL(loggingCamera_point()),this,SLOT(updateLogcamera_point()));
    //绑定快捷按钮相关函数
   //绑定slider的函数
  // connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_raw_valueChanged(int)));
  // connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(on_Slider_linear_valueChanged(int)));
   //设置界面
   connect(ui.action_2,SIGNAL(triggered(bool)),this,SLOT(slot_setting_frame()));

   //左工具栏tab索引改变
   connect(ui.tab_manager,SIGNAL(currentChanged(int)),this,SLOT(slot_tab_manage_currentChanged(int)));
   //右工具栏索引改变
     //刷新话题列表
    connect(ui.refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList()));



    //treewidget的值改变的槽函数

    //绑定treeview checkbox选中事件
   // stateChanged


    //connect(ui.treeWidget_rviz,SIGNAL(itemChanged(QTreeWidgetItem*,int)),this,SLOT(slot_treewidget_item_value_change(QTreeWidgetItem*,int)));
    connect(ui.pushButton_rgb_set,SIGNAL(clicked(bool)),this,SLOT(slot_set_rgb_config(bool)));
    connect(ui.pushButton_camera_set,SIGNAL(clicked(bool)),this,SLOT(slot_set_camera_config(bool)));

    connect(ui.pushButton_topic_camera,SIGNAL(clicked(bool)),this,SLOT(slot_topic_camera(bool)));
    connect(ui.pushButton_topic_rgb,SIGNAL(clicked(bool)),this,SLOT(slot_topic_stereo(bool)));

    connect(ui.pushButton_canny,SIGNAL(clicked(bool)),this,SLOT(slot_setup_canny_arg(bool)));
    connect(ui.pushButton_ROI,SIGNAL(clicked(bool)),this,SLOT(slot_setup_ROI_arg(bool)));
    connect(ui.pushButton_hough,SIGNAL(clicked(bool)),this,SLOT(slot_setup_hough_arg(bool)));
    connect(ui.pushButton_detection,SIGNAL(clicked(bool)),this,SLOT(slot_setup_detection_arg(bool)));



    connect(ui.comboBox_choose_pdf, SIGNAL(currentIndexChanged(int)),this,SLOT(on_pushButton_pdf_clicked(int)));
    connect(ui.comboBox_choose_tool, SIGNAL(currentIndexChanged(int)),this,SLOT(on_pushButton_tool_clicked(int)));

    //connect(ui.comboBox_can_cfg,SIGNAL(currentIndexChanged(int)),this,SLOT(slot_can_cfg(int)));


}

//设置界面
void MainWindow::slot_setting_frame()
{
    if(set!=NULL)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else{
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    //绑定set确认按钮点击事件
}
//刷新当前坐标
void MainWindow::slot_position_change(QString frame,double x,double y,double z,double w)
{

}

void MainWindow::displayCamera(const QImage &image)
{

}

void MainWindow::updateLogcamera()
{
  displayCamera(qnode.image);
}

void MainWindow::displayCamera_rgb(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_rgb = image.copy();
    qimage_mutex_.unlock();
}
void MainWindow::updateLogcamera_rgb()
{
  displayCamera_rgb(qnode.image_rgb);
}

void MainWindow::displayCamera_point(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_point = image.copy();
    qimage_mutex_.unlock();
}
void MainWindow::updateLogcamera_point()
{
    displayCamera_point(qnode.image_point);
}

void MainWindow::slot_can_cfg(int index)
{
    char yaml_set[200];
    char *yaml;
    int cfg_can;
    sprintf(yaml_set, "sudo ip link set can0 down");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);

    sprintf(yaml_set, "sudo ip link set can0 type can bitrate %d ",cfg_can);
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);

    sprintf(yaml_set, "sudo ip link set can0 up");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);
}

void MainWindow::slot_imu_rviz(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  roslaunch sanchi_amov rviz.launch ");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);
}

void MainWindow::slot_pushButton_laser2_set_2(bool checked)
{
    char yaml_set[200];
    char *yaml;
    
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);
}

void MainWindow::slot_gnss_rviz(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  roslaunch rtk_process gnss_rviz.launch ");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);
}

void MainWindow::slot_laser_rviz(bool checked)
{
    char yaml_set[200];
    char *yaml;
    

    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);

}

void MainWindow::slot_ars_rviz(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  roslaunch ars_40X ars_rviz.launch");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);



}

void MainWindow::slot_topic_gnss(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set,
            "gnome-terminal  --window -x  rostopic echo /gps/fix");
    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);
}

void MainWindow::slot_topic_laser(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set,
            "gnome-terminal  --window -x  rostopic echo  /lslidar_point_cloud");
    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);
}

void MainWindow::slot_topic_ars(bool checked)
{
    char yaml_set[200];
    char *yaml;

    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);
}

void MainWindow::slot_topic_imu(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set,
            "gnome-terminal  --window -x  rostopic echo  /imu/data_raw");
    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);

}

void MainWindow::slot_topic_stereo(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set, "gnome-terminal  --window -x  rostopic echo  /zkhy_stereo/left/color ");
    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);
}

void MainWindow::slot_topic_camera(bool checked)
{
    char yaml_set[200];
    char *yaml;
    sprintf(yaml_set, "gnome-terminal  --window -x  rostopic echo /usb_cam/image_raw");
    yaml= yaml_set;
    printf("-------------- %s %s----------\n", __func__, yaml);
    system(yaml);

}

void MainWindow::slot_topic_tof(bool checked)
{
    char yaml_set[400];
    char *yaml;

    sprintf(yaml_set, "gnome-terminal  --window -x  rostopic echo /tof_pub");
    yaml= yaml_set;
    system(yaml);
}

void MainWindow::slot_get_imu_bag_stop(bool checked)
{

}


void MainWindow::slot_get_imu_bag_record(bool checked)
{

}

void MainWindow::slot_get_imu_bag_play(bool checked)
{
	
}

void MainWindow::slot_update_tof()
{
   
}

void MainWindow::on_pushButton_pdf_clicked(int checked)
{
    int ret = 0;
    if ("Lidar" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince Lidar.pdf");
    }
    else if ("Radar" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince Radar.pdf");
    }
    else if ("Camera" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince Camera.pdf");
    }
    else if ("RGB" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince RGB.pdf");
    }
    else if ("IMU" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince IMU.pdf");
    }
    else if ("Ult" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince Ult.pdf");
    }
    else if ("GNSS" == ui.comboBox_choose_pdf->currentText())
    {
        ret = system("evince GNSS.pdf");
    }
    else
    {

    }

    if(ret)
    {
        QMessageBox::warning(this, QStringLiteral("提示"), QStringLiteral("PDF文档打开失败！error code：%1").arg(ret));
    }
}

void MainWindow::on_pushButton_tool_clicked(int)
{
    int ret = 0;
    char yaml_set[200];
    if ("cangaroo" == ui.comboBox_choose_tool->currentText())
    {
        sprintf(yaml_set, " /home/pc/work/scripts/cangaroo.sh");
        const char *yaml = yaml_set;
        printf("-------------- %s %s ----------\n", __func__, yaml);

        system(yaml);
    }
    else if ("Wireshark" == ui.comboBox_choose_tool->currentText())
    {
        ret = system("sudo wireshark");
    }
    else if ("Cutecom" == ui.comboBox_choose_tool->currentText())
    {
        ret = system("cutecom &");
    }
    else if ("Cheese" == ui.comboBox_choose_tool->currentText())
    {
        ret = system("cheese");
    }

    if(ret)
    {
        QMessageBox::warning(this, QStringLiteral("提示"), QStringLiteral("工具打开失败！error code：%1").arg(ret));
    }
}

void MainWindow::slot_button_cutecom(bool checked)
{
        int ret = 0;
        char yaml_set[200];
        sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  sudo cutecom");

        const char *yaml = yaml_set;
        printf("-------------- %s %s ----------\n", __func__, yaml);

        ret = system(yaml);

}
void MainWindow::slot_button_cutecom_2(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  sudo cutecom");

    const char *yaml = yaml_set;
    printf("-------------- %s %s ----------\n", __func__, yaml);

    ret = system(yaml);

}
void MainWindow::slot_button_cangoo(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, " /home/pc/work/scripts/cangaroo.sh");

    const char *yaml = yaml_set;
    printf("-------------- %s %s ----------\n", __func__, yaml);

    ret = system(yaml);
}

void MainWindow::slot_get_gnss_config(bool checked)
{
    int ret = 0;
    char yaml_get[200];
    sprintf(yaml_get, "rosparam get /move_base/global_costmap/height");

    const char *yaml = yaml_get;
    printf("-------------- %s %s ----------\n", __func__, yaml);

    ret = system(yaml);
}
void MainWindow::slot_get_imu_config(bool checked)
{
    printf("-------------- %s %d ----------\n", __func__, checked);

}
void MainWindow::slot_get_laser_config(bool checked)
{
  
}
void MainWindow::slot_get_laser2_config(bool checked)
{
    printf("-------------- %s %d ----------\n", __func__, checked);

}
void MainWindow::slot_get_rgb_config(bool checked)
{
    printf("-------------- %s %d ----------\n", __func__, checked);

}
void MainWindow::slot_get_camera_config(bool checked)
{
    printf("-------------- %s %d ----------\n", __func__, checked);

}
void MainWindow::slot_get_tof_config(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  /home/pc/work/scripts/cangaroo.sh");

    const char *yaml = yaml_set;
    printf("-------------- %s %s ----------\n", __func__, yaml);

    ret = system(yaml);

}

void MainWindow::slot_set_gnss_config(bool checked)
{
}

void MainWindow::slot_update_imu_data(double x,double y,double z)
{

}

void MainWindow::slot_update_gnss_data(GNSS_DATA msg)
{
}

void MainWindow::slot_update_lidar_data(LIDAR_DATA msg)
{

   
}

void MainWindow::slot_update_rgb_data(RGB_DATA msg)
{


}

void MainWindow::slot_set_imu_config(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, " roslaunch sanchi_amov imu_100D2.launch  &");

    const char *yaml = yaml_set;
    printf("-------------- %s %s ----------\n", __func__, yaml);

    ret = system(yaml);
}



void MainWindow::slot_set_laser_config(bool checked)
{

}

void MainWindow::slot_set_laser2_config(bool checked)
{
   
}

void MainWindow::slot_set_rgb_config(bool checked)
{
 ui.lineEdit_foc->setText("1.4821e-323");
 ui.lineEdit_cen_x->setText("2.9563e+15");
 ui.lineEdit_cen_y->setText("4.940e-324");
 ui.lineEdit_bias_x->setText("4.6667e-310");
 ui.lineEdit_bias_y->setText("4.6667e-310");
 ui.lineEdit_bias_z->setText("6.9527e-310");
}

void MainWindow::slot_set_camera_config(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  rosrun camera_calibration cameracalibrator.py --size %sx%s --square %s image:=/usb_cam/image_raw camera:=/usb_cam",
          ui.lineEdit_cam_x->text().toLocal8Bit().data(),ui.lineEdit_cam_x2->text().toLocal8Bit().data(),ui.lineEdit_cam_y->text().toLocal8Bit().data());

    const char *yaml = yaml_set;
   
  printf("-------------- %s %s ----------\n",  __func__, yaml_set);
    ret = system(yaml);
}

void MainWindow::slot_set_tof_config(bool checked)
{
    int ret = 0;
    char yaml_set[200];
    sprintf(yaml_set, "gnome-terminal --geometry=80x25+10+10  -x  /home/pc/work/scripts/cangaroo.sh");

  const char *yaml = yaml_set;
    printf("-------------- %s %s ----------\n",  __func__, yaml_set);

    ret = system(yaml);

}

void MainWindow::slot_update_ros_sense_data(SENSE_STATE sense_state)
{
        QPalette pa;
        QPalette pa2;
        pa.setColor(QPalette::WindowText, QColor(34,139,34));
        pa2.setColor(QPalette::WindowText, QColor(255,255,255));



        if (sense_state.camera_state == 1)
        {
            ui.label_camera_state->setPalette(pa);
            ui.label_camera_state->setStyleSheet("color:green;");
            ui.label_camera_state->setText("在线");
        }
        else
        {
            ui.label_camera_state->setPalette(pa2);
            ui.label_camera_state->setStyleSheet("color:red;");
            ui.label_camera_state->setText("离线");
        }

        if (sense_state.rgb_state == 1)
        {
            ui.label_binary_state->setPalette(pa);
            ui.label_binary_state->setStyleSheet("color:green;");
            ui.label_binary_state->setText("在线");
        }
        
        else
        {
            ui.label_binary_state->setPalette(pa2);
            ui.label_binary_state->setStyleSheet("color:red;");
            ui.label_binary_state->setText("离线");
        }

}


void MainWindow::update_sensor_data(Radar_STATE radar_data)
{
    QPalette pa;
    

if (send_id != radar_data.radar_id)
    send_id = radar_data.radar_id;


    tof_distance = radar_data.tof_l_dis;
    tof_r_distance = radar_data.tof_l_dis;


    if(radar_data.radar_dec_clu == 1)
    {
         // ui.label_type->setText("cluster");
    }
    else
    {
          //ui.label_type->setText("object");
    }
}


void MainWindow::slot_startUp_rgbd(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        printf("-------------------------------\n");
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun rqt_image_view rqt_image_view /zkhy_stereo/left/color");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {

    }

}

void MainWindow::slot_startUp_canny(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        system("rosnode kill /canny_node &");
        printf("-------------------------------\n");
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun camera_implementation canny_node");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {
        system("rosnode kill /canny_node &");
        printf("--------rosnode kill /canny_node--------\n");

    }

}

void MainWindow::slot_setup_canny_arg(bool checked)
{
    int max = ui.spinBox_canny_max->value();
    int min = ui.spinBox_canny_min->value();
    qnode.publishe_config_canny(max, min);
}

void MainWindow::slot_startUp_hough(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        system("rosnode kill /hough_node &");
        printf("-------------------------------\n");
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun camera_implementation hough_node");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {
        system("rosnode kill /hough_node &");
        printf("--------rosnode kill /hough_node--------\n");

    }

}


void MainWindow::slot_setup_hough_arg(bool checked)
{
    int max = ui.spinBox_hough_max->value();
    int min = ui.spinBox_hough_min->value();
    int hough_threshold = ui.spinBox_hough_hough->value();
    qnode.publishe_config_hough(max, min, hough_threshold);
}


void MainWindow::slot_startUp_ROI(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        system("rosnode kill /ROI_node &");
        printf("-------------------------------\n");
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun camera_implementation roi_node");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {
        system("rosnode kill /ROI_node &");
        printf("--------rosnode kill /ROI_node--------\n");

    }

}


void MainWindow::slot_setup_ROI_arg(bool checked)
{
    int x = ui.spinBox_ROI_x->value();
    int y = ui.spinBox_ROI_y->value();
    int w = ui.spinBox_ROI_w->value();
    int h = ui.spinBox_ROI_h->value();
    qnode.publishe_config_ROI(x, y, w, h);
}


void MainWindow::slot_startUp_detection(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        system("rosnode kill /detection_node &");
        printf("-------------------------------\n");
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun camera_implementation detection_node");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {
        system("rosnode kill /detection_node &");
        printf("--------rosnode kill /detection_node--------\n");

    }

}

void MainWindow::slot_setup_detection_arg(bool checked)
{
    std::string data_source = ui.comboBox_data->currentText().toStdString();
    std::string detect_type = ui.comboBox_type->currentText().toStdString();
    float iou = ui.lineEdit_iou->text().toFloat();
    float confidence = ui.lineEdit_confidence->text().toFloat();
    std::string cuda = ui.comboBox_cuda->currentText().toStdString();
    qnode.publishe_config_detection(data_source, detect_type, iou, confidence, cuda);
}


void MainWindow::slot_startUp_point(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        sprintf(yaml_get,
                "gnome-terminal --geometry=80x25+10+10 -x rosrun rqt_image_view rqt_image_view /zkhy_stereo/disparity");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
        //button_switch_rgbd->setChecked(false);
    }
    else
    {

    }

}

void MainWindow::slot_pushButton_can_cfg(bool checked)
{
    QPalette pa;


    char yaml_set[200];
    char *yaml;
    int cfg_can;

    sprintf(yaml_set, "sudo ip link set can0 down");
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);

    sprintf(yaml_set, "sudo ip link set can0 type can bitrate %d ",cfg_can);
    yaml= yaml_set;
    system(yaml);
    printf("-------------- %s %s----------\n", __func__, yaml);

    sprintf(yaml_set, "sudo ip link set can0 up");
    yaml= yaml_set;
    system(yaml);



    printf("-------------- %s %s----------\n", __func__, yaml);
}

void MainWindow::slot_startUp_gnss(bool checked)
{
    char yaml_get[200];
    if (1 == checked)
    {
        system("rosnode kill /rtk_process ");
        system("rosnode kill /nmea_topic ");
        sprintf(yaml_get, "roslaunch rtk_process rtk_process.launch &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {
        system("rosnode kill /rtk_process &");
        system("rosnode kill /nmea_topic &");   
    }

}
void MainWindow::slot_startUp_laser(bool checked)
{
    char yaml_get[200];

    if (1 == checked)
    {  
        system("rosnode kill /laser_link_to_world &");
        system("rosnode kill /lslidar_c16_decoder_node &");
        system("rosnode kill /lslidar_c16_driver_node &");

        sprintf(yaml_get, " roslaunch lslidar_c16_decoder lslidar_c16.launch &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {
        system("rosnode kill /laser_link_to_world &");
        system("rosnode kill /lslidar_c16_decoder_node &");
        system("rosnode kill /lslidar_c16_driver_node &");
    }
}
void MainWindow::slot_startUp_ars(bool checked)
{
    char yaml_get[200];

    if (1 == checked)
    {
        work->Thread_run();
    }
    else
    {
        //timer_tof->stop();
         work->Thread_stop();
    }

    if (1 == checked)
    {
        system("rosnode kill /ars_40X_ros &");
        system("rosnode kill /ars_40X_rviz &");

        sleep(1);

        sprintf(yaml_get, "roslaunch ars_40X ars_40X_rviz.launch  &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {
        system("rosnode kill /ars_40X_ros &");
        system("rosnode kill /ars_40X_rviz &");
    }
}

void MainWindow::slot_startUp_imu(bool checked)
{
    char yaml_get[200];

    if (1 == checked)
    {
        system("rosnode kill /imu &");

        sprintf(yaml_get, " roslaunch sanchi_amov imu_100D2.launch  &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {
        system("rosnode kill /imu &");
    }

}
void MainWindow::slot_startUp_stereo(bool checked)
{
    char yaml_get[200];
    printf("-------------- %s %d ----------\n", __func__, checked);

    if (1 == checked)
    {
        system("rosnode kill /stereo_listener &");
        system("rosnode kill /stereo_publisher &");

        sprintf(yaml_get,
               "roslaunch zkhy_stereo_d zkhy_stereo.launch  &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {        system("rosnode kill /stereo_listener &");
        system("rosnode kill /stereo_publisher &");
    }

}
void MainWindow::slot_startUp_camera(bool checked)
{
    char yaml_get[200];
    printf("-------------- %s %d ----------\n", __func__, checked);

    if (1 == checked)
    {
        system("rosnode kill /usb_cam &");
        sleep(1);
        sprintf(yaml_get, "roslaunch usb_cam usb_cam-test.launch  &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);
    }
    else
    {
        system("rosnode kill /usb_cam &");
    }

}
void MainWindow::slot_startUp_tof(bool checked)
{
    if (1 == checked)
    {
        work->Thread_run();
    }
    else
    {
        //timer_tof->stop();
         work->Thread_stop();
    }
}

//刷新返航地点
void MainWindow::slot_set_return_point()
{


}
//返航
void MainWindow::slot_return_point()
{

}
//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz->Set_Pos();
// ui.label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz->Set_Goal();
//  ui.label_map_msg->setText("请在地图中选择机器人导航的目标位置");
}
void MainWindow::slot_move_camera_btn()
{
    map_rviz->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz->Set_Select();
}
//treewidget的checkbox是否选中槽函数
void MainWindow::slot_treewidget_item_check_change(int is_check)
{
    QCheckBox* sen = (QCheckBox*)sender();
    qDebug()<<"check:"<<is_check<<"parent:"<<widget_to_parentItem_map[sen]->text(0)<<"地址："<<widget_to_parentItem_map[sen];
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString dis_name=widget_to_parentItem_map[sen]->text(0);
    bool enable=is_check>1?true:false;
    if(dis_name=="Grid")
    {



    }
    else if(dis_name=="Map")
    {

    }
    else if(dis_name=="LaserScan")
    {

    }
    else if(dis_name=="Navigate")
    {

    }
    else if(dis_name=="RobotModel")
    {
        map_rviz->Display_RobotModel(enable);
    }
}
//treewidget 的值改变槽函数
void MainWindow::slot_treewidget_item_value_change(QString value)
{

    QWidget* sen = (QWidget*)sender();
    qDebug()<<sen->metaObject()->className()<<"parent:"<<widget_to_parentItem_map[sen]->text(0);
    qDebug()<<value;
    QTreeWidgetItem *parentItem=widget_to_parentItem_map[sen];
    QString Dis_Name=widget_to_parentItem_map[sen]->text(0);

//    qDebug()<<"sdad"<<enable;
    //判断每种显示的类型
    if(Dis_Name=="Grid")
    {


    }
    else if(Dis_Name=="Global Options")
    {

    }
    else if(Dis_Name=="Map")
    {

    }
    else if(Dis_Name=="LaserScan")
    {

    }


}

void MainWindow::slot_pushButton_tof_update(bool check)
{
     QPalette pa;
    if((tof_distance>0) && (button_switch_tof->getChecked() == 1))
    {
        pa.setColor(QPalette::WindowText, QColor(34,139,34));
    }
    else
    {
        pa.setColor(QPalette::WindowText, QColor(255,139,0));

    }
}

//rviz添加topic的槽函数
void MainWindow::slot_add_topic_btn()
{

 
}
//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose)
{

}
//左工具栏索引改变
void MainWindow::slot_tab_manage_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:


        break;
    case 2:
        break;

    }
}
//右工具栏索引改变
void MainWindow::slot_tab_Widget_currentChanged(int index)
{
    switch (index) {
    case 0:

        break;
    case 1:
        ui.tab_manager->setCurrentIndex(1);
        break;
    case 2:
        break;

    }
}
//速度控制相关按钮处理槽函数
void MainWindow::slot_cmd_control()
{
}
//滑动条处理槽函数
void MainWindow::on_Slider_raw_valueChanged(int v)
{

}
//滑动条处理槽函数
void MainWindow::on_Slider_linear_valueChanged(int v)
{

}
//快捷指令删除按钮
void MainWindow::quick_cmd_remove()
{



}
//快捷指令添加按钮
void MainWindow::quick_cmd_add()
{
    QWidget *w=new QWidget;
    //阻塞其他窗体
    w->setWindowModality(Qt::ApplicationModal);
    QLabel *name=new QLabel;
    name->setText("名称:");
    QLabel *content=new QLabel;
    content->setText("脚本:");
    QLineEdit *name_val=new QLineEdit;

    QPushButton *ok_btn=new QPushButton;
    ok_btn->setText("ok");
    ok_btn->setIcon(QIcon("://images/ok.png"));
    QPushButton *cancel_btn=new QPushButton;
    cancel_btn->setText("cancel");
    cancel_btn->setIcon(QIcon("://images/false.png"));
    QHBoxLayout *lay1=new QHBoxLayout;
    lay1->addWidget(name);
    lay1->addWidget(name_val);
    QHBoxLayout *lay2=new QHBoxLayout;
    lay2->addWidget(content);

    QHBoxLayout *lay3=new QHBoxLayout;
    lay3->addWidget(ok_btn);
    lay3->addWidget(cancel_btn);
    QVBoxLayout *v1=new QVBoxLayout;
    v1->addLayout(lay1);
    v1->addLayout(lay2);
    v1->addLayout(lay3);

    w->setLayout(v1);
    w->show();

}
//向treeWidget添加快捷指令
void MainWindow::add_quick_cmd(QString name,QString val)
{
    if(name=="") return;
    QTreeWidgetItem *head=new QTreeWidgetItem(QStringList()<<name);

    QCheckBox *check=new QCheckBox;
    //记录父子关系
    this->widget_to_parentItem_map[check]=head;
    //连接checkbox选中的槽函数
    connect(check,SIGNAL(stateChanged(int)),this,SLOT(quick_cmds_check_change(int)));

    QTreeWidgetItem *shell_content=new QTreeWidgetItem(QStringList()<<"shell");

    head->addChild(shell_content);

}
//快捷指令按钮处理的函数
void MainWindow::quick_cmds_check_change(int state)
{
    QCheckBox* check = qobject_cast<QCheckBox*>(sender());
    QTreeWidgetItem *parent=widget_to_parentItem_map[check];

    bool is_checked=state>1?true:false;
    if(is_checked)
    {

    }
    else{


    }

}
//执行一些命令的回显
void MainWindow::cmd_output()
{

}
//执行一些命令的错误回显
void MainWindow::cmd_error_output()
{

}

//析构函数
MainWindow::~MainWindow() {
    char yaml_get[200];

    if( base_cmd)
    {
        delete base_cmd;
        base_cmd=NULL;
    }
    if(map_rviz)
    {
        delete map_rviz;
        map_rviz=NULL;
    }

  sprintf(yaml_get,
    "gnome-terminal  -x rosnode kill --all &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);

    delete media_player;
    delete  button_switch_camera;
    delete  button_switch_rgb;

    delete button_switch_ROI;
    delete button_switch_canny;
    delete button_switch_detection;
    delete button_switch_hough;

    delete  button_switch_rgbd;
    delete button_switch_point;

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check) {
    //如果使用环境变量
	if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() )
        {
            //showNoMasterMessage();
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
            ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");

            ui.label_camera_state->setStyleSheet("color:red;");
            ui.label_binary_state->setStyleSheet("color:red;");
            ui.label_camera_state->setText("离线");
            ui.label_binary_state->setText("离线");
        
            ui.tab_manager->setTabEnabled(1,false);
        
        }
        else
        {
            ui.tab_manager->setTabEnabled(1,true);
    

            //初始化rviz
            initRviz();

            ui.button_connect->setEnabled(false);
            ui.label_statue_text->setStyleSheet("color:green;");
            ui.label_statue_text->setText("在线");
            //初始化视频订阅的显示
            initVideos();
            //显示话题列表
            initTopicList();
		}
    }
    //如果不使用环境变量
    else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
            QMessageBox::warning(NULL, "失败", "连接ROS Master失败！请检查你的网络或连接字符串！", QMessageBox::Yes , QMessageBox::Yes);
             ui.label_statue_text->setStyleSheet("color:red;");
            ui.label_statue_text->setText("离线");

            ui.label_camera_state->setStyleSheet("color:red;");
            ui.label_binary_state->setStyleSheet("color:red;");
            ui.label_camera_state->setText("离线");
            ui.label_binary_state->setText("离线");

            ui.tab_manager->setTabEnabled(1,false);
        

            //showNoMasterMessage();
		} else {
            ui.tab_manager->setTabEnabled(1,true);
    
            //初始化rviz
            initRviz();

			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			//ui.line_edit_topic->setReadOnly(true);

            ui.label_statue_text->setStyleSheet("color:green;");
           ui.label_statue_text->setText("在线");
            //启动参数配置节点
           //初始化视频订阅的显示
           initVideos();
           //显示话题列表
           initTopicList();
		}
	}

}

QString MainWindow::getUserName()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}

void MainWindow::on_pushButton_rvizReadDisplaySet_clicked()
{
    if (map_rviz == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "导入 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        map_rviz->ReadDisplaySet(path);
    }
}

void MainWindow::on_pushButton_rvizSaveDisplaySet_clicked()
{
    if (map_rviz == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "导出 RVIZ Display 配置", "/home/" + getUserName() + "/", "YAML(*.yaml)");

    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        map_rviz->OutDisplaySet(path);
    }
}

void MainWindow::initTopicList()
{
    ui.topic_listWidget->clear();
    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
        ui.topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
        printf("------%s------\n",iter.value());
        //if ("/rosout_agg" ==  QString("%1").arg(iter.key()))
        //ui.label->setText(QString("%1").arg(iter.key()));
    }
}
void MainWindow::refreashTopicList()
{
    initTopicList();
}
//当ros与master的连接断开时
void MainWindow::slot_rosShutdown()
{

     ui.label_statue_text->setStyleSheet("color:red;");
    ui.label_statue_text->setText("离线");
    ui.button_connect->setEnabled(true);
    ui.line_edit_master->setReadOnly(false);
    ui.line_edit_host->setReadOnly(false);
    //ui.line_edit_topic->setReadOnly(false);
}
void MainWindow::slot_power(float p)
{

}
void MainWindow::slot_speed_x(double x)
{

}
void MainWindow::slot_speed_y(double x)
{

}
void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        //ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    //QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_monitor");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }

    //读取快捷指令的setting
    QSettings quick_setting("quick_setting","cyrobot_monitor");
    QStringList ch_key=quick_setting.childKeys();
    for(auto c:ch_key)
    {
        add_quick_cmd(c,quick_setting.value(c,QString("")).toString());
    }

}

void MainWindow::WriteSettings() 
{
    QSettings settings("Qt-Ros Package", "cyrobot_monitor");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));


}

void MainWindow::closeEvent(QCloseEvent *event)
{

	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor

void cyrobot_monitor::MainWindow::on_actionquit_triggered()
{
    char yaml_get[200];


  sprintf(yaml_get,
    "gnome-terminal  -x rosnode kill --all &");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);


      this->close();

}

void cyrobot_monitor::MainWindow::on_pushButton_clicked()
{
    ui.label_statue_text->setStyleSheet("color:red;");
   ui.label_statue_text->setText("离线");
   ui.button_connect->setEnabled(true);
   ui.line_edit_master->setReadOnly(false);
   ui.line_edit_host->setReadOnly(false);
   //ui.line_edit_topic->setReadOnly(false);
}

void cyrobot_monitor::MainWindow::on_pushButton_2_clicked()
{
    char yaml_get[200];


  sprintf(yaml_get,
    "gnome-terminal --geometry=80x25+10+10 -x rosnode kill --all");
        printf("-------------- %s %s ----------\n", __func__, yaml_get);
        system(yaml_get);


      this->close();
}

void cyrobot_monitor::MainWindow::on_pushButton_3_clicked()
{
    char yaml_get[200];

    sprintf(yaml_get,
            "gnome-terminal --geometry=80x25+10+10 -x rosrun rqt_image_view rqt_image_view /usb_cam/image_raw");
    printf("-------------- %s %s ----------\n", __func__, yaml_get);
    system(yaml_get);
}
