  /**@file
 * @note BITCo., Ltd. All rights reserved.
 * @brief 线程及传感器监控
 *
 * @author baimatanhua
 * @date 2023/09/16
 *
 * @version
 *  date        |version |author              |message
 *  :----       |:----   |:----               |:------
 *  2023/09/16  |V1.0.0  |baimatanhua          |进程及传感器监控
 * @warning 
 */

#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/MapMetaData.h>

template <typename MsgType>
class NodeMonitor 
{
public:
    NodeMonitor(const std::string& node_name, const std::string& topic, int check_interval)
        : node_name_(node_name), topic_(topic), check_interval_(check_interval), node_exists_(false), topic_exists_(false)
    {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 创建订阅器
        sub_ = nh.subscribe(topic_, 1, &NodeMonitor::callback, this);

        // 创建发布器
        pub_ = nh.advertise<std_msgs::String>("/node_monitor", 1);

        // 创建定时器，周期性检测节点存在性和话题存在性
        timer_ = nh.createTimer(ros::Duration(check_interval_), &NodeMonitor::checkExistence, this);
    }

    void callback(const typename MsgType::ConstPtr& msg) 
    {
        // 在此处处理数据
        topic_exists_ = true;
    }

    void checkExistence(const ros::TimerEvent& event)
    {
        std_msgs::String msg;
        msg.data = node_name_ + "," + (isNodeExist() ? "yes" : "no") + "," + (topic_exists_ ? "yes" : "no");

        pub_.publish(msg);

        topic_exists_ = false;


    }

    bool isNodeExist()
    {
        std::string node_name = "/" + node_name_;  // 加上斜杠以符合 ROS 节点的命名规范
        std::vector<std::string> nodes;
        ros::master::getNodes(nodes);

        for (const auto& node : nodes) 
		{
            if (node == node_name) 
			{
                node_exists_ = true;
				ROS_WARN("[%s] Node does  exist.", node_name_.c_str());
                return true;
            }
        }

        ROS_WARN("[%s] Node does not exist.", node_name_.c_str());
	    return false;
    }

private:
    std::string node_name_;
    std::string topic_;
    int check_interval_;
    bool node_exists_;
    bool topic_exists_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) 
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "monitor_node");

    // 创建 NodeMonitor 对象，监控不同节点和话题
    NodeMonitor<sensor_msgs::LaserScan> laser_monitor("rplidarNode", "/scan", 3);                                            // 每秒3检测一次
    NodeMonitor<sensor_msgs::Imu> imu_monitor("ahrs_driver", "/imu_data", 3);                                                  // 每3秒检测一次
    NodeMonitor<nav_msgs::MapMetaData> icp_local_monitor("icp_localization_node", "/map_metadata", 3);  // 每3秒检测一次
    NodeMonitor<nav_msgs::MapMetaData> agv_control_monitor("agv_control_node", "/patrol_task_state", 3);  // 每3秒检测一次
    // 添加更多节点和话题监控...

    // 进入 ROS 主循环
    ros::spin();

    return 0;
}