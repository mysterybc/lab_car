# ndt_localization
用ndt_omp来进行scan to map 的用已知地图的激光lidar定位

激光lidar：vlp16 imu：惯导imu roll pitch yaw

程序依赖：
ndt_omp octomap_ros octomap_msg  

代码中可能存在的问题：


运行方式：
‘’‘
roslaunch add_pcl_link run.launch
’‘’
打开自己的数据集
‘’‘
rosbag play --clock ×××.bag
’‘’
加入了octomap生成方式

从以下两个方向进行改进：

1、octomap的点云进行滤波且不影响观测
2、将octomap压缩成为栅格地图