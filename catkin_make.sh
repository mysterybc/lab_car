#! /bin/bash

#请在工程文件中搜索"melodic devel"并注释掉下面的代码，取消注释"kinetic devel"下面的代码
# kinetic版本编译
git submodule init && git submodule update
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_msgs;robot_library;apriltag_ros" -DCATKIN_BLACKLIST_PACKAGES=""
gnome-terminal -x "roscore"
sleep 1s
./devel/lib/robot_library/change_ros_version Kinetic
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="apriltag_ros"



#请在工程文件中搜索"kinetic devel"并注释掉下面的代码，取消注释"melodic devel"下面的代码
#kinetic 版本的costmap_2dmelodic版本无法使用，并且path_coverage包会冲突
# git submodule init && git submodule update
# cd ..
# catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_msgs;robot_library;apriltag_ros" -DCATKIN_BLACKLIST_PACKAGES=""
# gnome-terminal -x "roscore"
# sleep 1s
# ./devel/lib/robot_library/change_ros_version Kinetic
# catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="apriltag_ros"
