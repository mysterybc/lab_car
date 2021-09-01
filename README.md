# 项目介绍

### 1、项目简介

- 代码为实验室项目无人车部分代码，不包含上位机。
- 仿真包含stage环境和gazebo环境。

### 2、安装依赖

- 安装ZeroMQ

  ```bash
  1、安装必要的包
  sudo apt-get install libtool pkg-config build-essential autoconf automake
  
  2、克隆安装zmq核心库（不包含其他）
  git clone https://github.com/zeromq/libzmq
  cd libzmq
  ./autogen.sh
  ./configure && make check
  sudo make install
  sudo ldconfig
  ```

- 安装ros依赖

  ```bash
  sudo apt-get install -y ros-${ROS_DISTRO}-move-base \
  						ros-${ROS_DISTRO}-move-base-msgs \
  						ros-${ROS_DISTRO}-global-planner \
  						ros-${ROS_DISTRO}-teb-local-planner \
  						ros-${ROS_DISTRO}-serial \
  						ros-${ROS_DISTRO}-map-server \
  						ros-${ROS_DISTRO}-pointcloud-to-laserscan \
  						ros-${ROS_DISTRO}-joy \
  						ros-${ROS_DISTRO}-octomap-ros\
  						ros-${ROS_DISTRO}-amcl
  ```

- 安装Realsense驱动

  [安装链接](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

- 安装apriltag

  ```bash
  cd {YOUR_LIB_PATH}
  git clone https://github.com/AprilRobotics/apriltag.git
  cd apriltag
  mkdir build && cd build
  cmake ..
  sudo make install

### 3、编译程序

- 首先编译依赖（请在工作空间中运行）

  `catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_msgs;robot_library;apriltag_ros" -DCATKIN_BLACKLIST_PACKAGES=""`

- 再编译全部文件，注意kinetic和melodic有一些区别

  目前change_ros_version包有bug，需要手动修改，搜索所有kinetic devel注释，修改为对应版本

  - service_node/path_coverage/src/CleaningPathPlanner.cpp  56-61行
  - service_node/path_coverage/src/LocalGridVelocityComputer.cpp 52-57行
  - service_node/path_coverage/src/PathPlanningNode.cpp 145-149行

- Kinetic

  `catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="apriltag_ros"`

- melodic (costmap2d是kinetic版本的)

  `catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="apriltag_ros;costmap_2d;"`

### 4、运行程序

- 实车：

  - 遥控： 

    确认遥控器输入通道正确。

    `roslaunch bring_up remote_control.launch` 

  - 全部功能：

    配置launch文件中所有全局参数（说明在文件中）。

    `roslaunch bring_up single_car.launch`

- 仿真：

  - 单车stage：

    `roslaunch bring_up stage_singlecar.launch `

  - 四车stage：

    `roslaunch bring_up stage_fourcar.launch `

  - 单车Gazebo：

    `roslaunch bring_up gazebo_singlecar.launch`

  - 两车Gazebo：

    `roslaunch bring_up gazebo_multicar.launch`

  - 运行控制程序

    `rosrun simulation host_cmd_pub`

### 4、说明

- 目前imu只通过角速度积分的方式接入了path_follow和laser march，move_base没有使用
- 

### 5、文件结构

- 目前完成的内容包含了7种节点，仿真环境以及需要的消息类型。

- 7种节点包含：控制节点、决策节点、感知节点、服务节点、资源节点、任务节点和通讯节点。

- 仿真环境基于ros stage和Gazebo实现。

- robot_msgs包含通讯所需msg，service and action三种通讯消息。

### 6、目前可实现功能

- 可以运行决策树，结合规划服务、路径跟随以及方针，能够实现四车的集结、编队和搜索。

### 7、节点情况

###### （1）资源节点： 已经完成资源节点编写，实车均已测试。

###### （2）控制节点： 遥控节点目前可以遥控仿真小车，实车已经测试。

###### （3）服务节点： 可以实现简单的散点算法，需要双车运行在同一个roscore下面。	

###### （4）决策节点： 实现基于[BEHAVIAC](https://www.behaviac.com/language/zh/%e9%a6%96%e9%a1%b5/)框架的决策树，目前只有集结和等待任务。		

###### （5）任务节点： 可以实现仿真小车的集结任务。

###### （6）感知节点： 目前感知节点没有作用。

###### （7）通讯节点： 目前通讯节点完成了上位机交互初步功能。

###### （8）定位节点：ndt_localization, 基于ndt点云匹配的定位算法。

###### （9）bitrobot：行军控制库

### 8、提交记录

###### 2020年9月11日提交     

（1）加入散点算法 （2）加入上位机交互的通讯节点 （3）精简仿真代码

###### 2020年9月21日提交

（1）加入costmap  （2）加入icra中基于teb实现的local_planner，但是目前存在问题

###### 2020年10月7日提交

（1）区分了两辆车的参数（2）测试了双车集结任务（3）优化了定位初始化方式（4）上位机通讯已测试

###### 2020年10月20日提交

（1）添加了集结任务中的上位机交互 （2）调整代码 （3）修正激光雷达最小测量距离，避开天线杆臂

（4）添加了散点算法对于动态车辆数量的调整（5）添加了双车通讯，散点，仿真测试正确

###### 2020年10月24号提交

（1）上报位姿使用tf，避免修改topic（2）修改一些bug

###### 2020年10月30号提交

（1）更换behaviac决策树（2）修改一些集结任务的bug（3）更改mission格式

###### 2020年11月3日提交

（1）加入了Gazebo仿真，针对多车加入了namespace，将tf的frame通过param传入

（2）修改host_cmd_pub以及host_cmd.msg，支持多任务。

###### 2020年11月6日提交

（1）修改决策树在多任务下的逻辑（2）加入多车协同行进控制BitCtrl（3）加入上位机遥控

 2020年12月22日提交

（1）修改通讯节点策略，调整机器人通讯格式和信息，加入group msg

（2）加入行军任务，基于bitctrl，分别使用gps和激光进行定位

（3）测试了gps和imu节点，可以使用gps进行定位，将imu融合到了lidar定位中

（4）添加了Debug专用格式  （5）修改了很多bug 。。。

