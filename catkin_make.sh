#! /bin/bash

#请在工程文件中搜索"melodic devel"并注释掉下面的代码，取消注释"kinetic devel"下面的代码
# kinetic版本编译
#cd ..
#catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_msgs" -DCATKIN_BLACKLIST_PACKAGES=""
#catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_library" -DCATKIN_BLACKLIST_PACKAGES=""
#catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES=""



#请在工程文件中搜索"kinetic devel"并注释掉下面的代码，取消注释"melodic devel"下面的代码
#kinetic 版本的costmap_2dmelodic版本无法使用，并且path_coverage包会冲突
#melodic版本编译
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_msgs" -DCATKIN_BLACKLIST_PACKAGES=""
catkin_make -DCATKIN_WHITELIST_PACKAGES="robot_library" -DCATKIN_BLACKLIST_PACKAGES=""
catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCATKIN_BLACKLIST_PACKAGES="costmap_2d"
