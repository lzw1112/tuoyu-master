/**
 * @file /include/cyrobot_monitor/main_window.hpp
 *
 * @brief Qt based gui for cyrobot_monitor.
 *
 * @date November 2010
 **/
#ifndef cyrobot_monitor_MAIN_WINDOW_H
#define cyrobot_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include <QTimer>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "addtopics.h"
#include "settings.h"
#include "qrviz.hpp"
//仪表盘头文件
#include "CCtrlDashBoard.h"
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QSoundEffect>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool.h>
#include "controlcan.h"
#include "dialog.h"

#include "switchbutton.h"
/*****************************************************************************
** Namespace
*****************************************************************************/
namespace cyrobot_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT
private:
    Ui::MainWindowDesign ui;
    CanThread* work;
    QThread* subthread;

    QColor block;
    QColor green;
    int can_cfg_flag;
    int time_calib;
    typedef struct
    {
        double acc_x;
        double acc_y;
        double acc_z;
    }ACC_DATA;

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

    void connections();
    void add_quick_cmd(QString name,QString shell);
    QNode qnode;
    CCtrlDashBoard *m_DashBoard_x;
    CCtrlDashBoard *m_DashBoard_y;
    QProcess *quick_cmd=NULL;
    QProcess *close_remote_cmd=NULL;
    QProcess *base_cmd=NULL;
    QRviz *map_rviz=NULL;
    QStandardItemModel* treeView_rviz_model=NULL;
    AddTopics *addtopic_form=NULL;
    //存放rviz treewidget当前显示的控件及控件的父亲的地址
    QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map;
    //存放状态栏的对应关系 display名 状态item
    QMap <QString,QTreeWidgetItem *> tree_rviz_stues;
    //存放display的当前值 item名，参数名称和值
    QMap <QTreeWidgetItem*,QMap<QString,QString>> tree_rviz_values;
    Settings *set=NULL;
    QSoundEffect *media_player=NULL;
    SwitchButton *button_switch_laser;
    SwitchButton *button_switch_laser2;
    SwitchButton *button_switch_camera;
    SwitchButton *button_switch_rgb;
    SwitchButton *button_switch_gnss;
    SwitchButton *button_switch_imu;
    SwitchButton *button_switch_tof;

    SwitchButton *button_switch_rgbd;
    SwitchButton *button_switch_point;

    int laser_state;
    int laser2_state;
    int camera_state;
    int rgb_state;
    int gnss_state;
    int imu_state;
    int timerId1;
    int timerId2;
    int tof_dis;
    int nbytes;
    QTimer *timer_tof;
    QImage qimage_;
    QImage qimage_rgb;
    QImage qimage_point;
    mutable QMutex qimage_mutex_;
    QMap<QString, QString> m_mapRvizDisplays;
    QAbstractItemModel* m_modelRvizDisplay;
    QString m_sRvizDisplayChooseName_;
    int send_id;
    int dec_type;
    int tof_distance;
    int tof_r_distance;
    int gnss_cnt;
    int pub_cnt;
public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void initRviz();
    void initUis();
    void initData();
    void initVideos();
    void initTopicList();

    QString JudgeDisplayNewName(QString name);
protected:
    void mouseDoubleClickEvent (QMouseEvent *event);
public slots:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();

    /// \brief 导入rviz Display配置
    void on_pushButton_rvizReadDisplaySet_clicked();
    /// \brief 导出rviz Display配置
    void on_pushButton_rvizSaveDisplaySet_clicked();
    QString getUserName();
    void on_button_connect_clicked(bool check );
    void RvizGetModel(QAbstractItemModel *model);
    void on_pushButton_pdf_clicked(int);
    void on_pushButton_tool_clicked(int);

    void slot_pushButton_tof_update(bool check);

    void on_checkbox_use_environment_stateChanged(int state);
    void slot_speed_x(double x);
    void slot_speed_y(double y);
    void slot_power(float p);
    void slot_rosShutdown();
    void quick_cmds_check_change(int);
    void cmd_output();
    void cmd_error_output();
    void refreashTopicList();
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void on_Slider_raw_valueChanged(int value);
    void on_Slider_linear_valueChanged(int value);
    void slot_cmd_control();
    void slot_tab_manage_currentChanged(int);
    void slot_tab_Widget_currentChanged(int);
    void slot_add_topic_btn();
    void slot_choose_topic(QTreeWidgetItem *choose);
    void slot_treewidget_item_value_change(QString);
    void slot_treewidget_item_check_change(int);
    void slot_set_2D_Goal();
    void slot_set_2D_Pos();
    void slot_set_select();
    void slot_move_camera_btn();
    //设置界面
    void slot_setting_frame();
    //设置返航点
    void slot_set_return_point();
    //返航
    void slot_return_point();
    //机器人位置
    void slot_position_change(QString,double,double,double,double);
    void quick_cmd_add();
    void quick_cmd_remove();
    //显示图像
    //显示图像
    void slot_show_image(int,QImage);
//    void on_horizontalSlider_raw_valueChanged(int value);

    //launch 启动项
    void slot_startUp_gnss(bool checked);
    void slot_startUp_laser(bool checked);
    void slot_startUp_ars(bool checked);
    void slot_startUp_imu(bool checked);
    void slot_startUp_stereo(bool checked);
    void slot_startUp_camera(bool checked);
    void slot_startUp_tof(bool checked);


    void slot_startUp_rgbd(bool checked);
    void slot_startUp_point(bool checked);

    void slot_pushButton_can_cfg(bool checked);

    //topic 启动项
    void slot_topic_gnss(bool checked);
    void slot_topic_laser(bool checked);
    void slot_topic_ars(bool checked);
    void slot_topic_imu(bool checked);
    void slot_topic_stereo(bool checked);
    void slot_topic_camera(bool checked);
    void slot_topic_tof(bool checked);

    //set 设置项
    void slot_set_gnss_config(bool checked);
    void slot_set_imu_config(bool checked);

    void slot_set_laser_config(bool checked);
    void slot_set_laser2_config(bool checked);
    void slot_set_rgb_config(bool checked);
    void slot_set_camera_config(bool checked);
    void slot_set_tof_config(bool checked);

    void slot_get_gnss_config(bool checked);
    void slot_get_imu_config(bool checked);

    void slot_button_cutecom(bool checked);
    void slot_button_cutecom_2(bool checked);
    void slot_button_cangoo(bool checked);

    void slot_get_imu_bag_record(bool checked);
 void slot_get_imu_bag_stop(bool checked);
    void slot_get_imu_bag_play(bool checked);
    void slot_can_cfg(int);
    void slot_get_laser_config(bool checked);
    void slot_get_laser2_config(bool checked);
    void slot_get_rgb_config(bool checked);
    void slot_get_camera_config(bool checked);
    void slot_get_tof_config(bool checked);
    void slot_update_tof();

    void updateLogcamera();
    void displayCamera(const QImage& image);
    void updateLogcamera_rgb();
    void displayCamera_rgb(const QImage& image);
    void updateLogcamera_point();
    void displayCamera_point(const QImage& image);

    void update_sensor_data(Radar_STATE radar_data);


   void slot_imu_rviz(bool checked);
   void slot_gnss_rviz(bool checked);
   void slot_laser_rviz(bool checked);
   void slot_ars_rviz(bool checked);


//rviz
    void slot_pushButton_laser2_set_2(bool checked);

private slots:
    void slot_update_imu_data(double,double,double);
    void slot_update_gnss_data(GNSS_DATA);
    void slot_update_lidar_data(LIDAR_DATA);
    void slot_update_rgb_data(RGB_DATA);

    void slot_update_ros_sense_data(SENSE_STATE);

    void on_actionquit_triggered();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
};
}// namespace cyrobot_monitor

#endif // cyrobot_monitor_MAIN_WINDOW_H
