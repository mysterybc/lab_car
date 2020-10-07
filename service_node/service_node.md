# 服务节点

### 1、散点

###### （1）节点介绍

​		散点节点通过ROS Service，接受集结目标点和四个机器人位置，分配集结位置。

​		注：目前还没有添加避开障碍物。

###### （2）输入

​			名称						类型							名称				

​		机器人位姿    			Topic  					"odom"

​			地图						Topic					  "map"

​			目标点				Service			    "separate_goal"

###### （4）输出

​			名称						类型							名称				

​			目标点				Service			    "separate_goal"





​		

​		