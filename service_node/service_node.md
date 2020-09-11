# 服务节点

### 1、规划节点

###### （1）节点介绍

​		规划节点通过ROS Service的方式，接受小车实时位置信息、目标位置和地图信息，根据A Star算法计算出目标路径，并发布。

​		注：目前新的目标点不会打断上一次规划。

​				目前接受静态地图，没有感知信息。

###### （2）实现算法 

​		 A Star

###### （3）输入

​			名称						类型							名称				

​		机器人位姿    			Topic  					"odom"

​			地图						.jpg						file_path

​			目标点				Service			"global_planning"

###### （4）输出

​			名称						类型							名称				

​			路径					Service				"global_planning"

### 2、散点

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