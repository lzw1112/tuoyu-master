# introduction
本功能包为拓渝科技有限公司的智能网联传感器教学软件，
可用于雷达、相机、组合惯导传感器教学；
# install 
1. 安装ros
2. 下载功能包
    $ cd ~
    $ mkdir -p tuoyu_ws/src
    $ cd tuoyu_ws/src
    $ catkin_init_workspace
    $ cd .. 
    $ catkin build 
    $ cd src
    $ git clone git@github.com:ZhawedHoo1998:tuoyu_sensorteaching.git
3. 安装相关依赖
    $ sudo apt-get install build-essential git qt5-qmake qtbase5-dev libnl-3-dev libnl-route-3-dev ros-melodic-costmap-converter ros-melodic-velodyne ros-melodic-grid-map ros-melodic-jsk-rviz-plugins cutecom cheese  libdw-dev ros-melodic-nmea-msgs carla-ros-bridge
    $ sudo apt-get install qtmultimedia5-dev

    sudo apt-get install -y build-essential git qt5-qmake qtbase5-dev libnl-3-dev 
  libnl-route-3-dev ros-noetic-costmap-converter ros-noetic-velodyne            
  ros-noetic-grid-map ros-noetic-jsk-rviz-plugins cutecom cheese libdw-dev
  ros-noetic-nmea-msgs qtmultimedia5-dev liblapack-dev libsuitesparse-dev       
  libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
   

    **安装ceres，用于imu标定**
    $ sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
    $ cd ~/tuoyu_ws/src/tuoyu_sensorteaching/driver/ceres-solver
    $ mkdir build
    $ cd build
    **快速升级cmake**
    $ pip install cmake==3.17.3
    $ sudo rm -rf /usr/bin/cmake
    $  which cmake
    $ sudo ln -s xxx /usr/bin/cmake
    **重新编译**
    $ cmake ..
    $ make -j4
    $ sudo make install
    **不跟踪build**
    $ catkin build code_utils
    $ catkin build imu_utils
    




4. 继续编译
cd ..
catkin build 

# 源码安装cangaroo —— 如何使用
cd ~/tuoyu_ws/src/tuoyu/driver/cangaroo
qmake -qt=qt5
make
make install
sudo cp ./bin/cangaroo /usr/local/bin/
# ppa安装wireshark
sudo add-apt-repository ppa:wireshark-dev/stable
sudo apt update
sudo apt install wireshark
## 安装过程选择是，允许非超级用户组捕获数据包
sudo usermod -aG wireshark $(whoami)
reboot

5. 编译运行
catkin_make

# 功能开发
# 1. camera 
    a. 基本功能实现

# 二次开发
1.  安装QT-ROS开发环境
参考： https://www.guyuehome.com/6058
2. 安装ros workspace相关插件
参考： https://blog.csdn.net/m0_62534602/article/details/129925756