# Porting libbitctrl to ROS and Ubuntu

Version 0.0.2, 2020/11/02

## Structre
1. config/ config files for libbitctrl.
2. doc/ documents (TBD)
3. cpp/ code of bitctrl, simurobot, ssnet, and GeographicLib (3rd party)
4. catkin_ws/src/  sample ros projects

## How to build
(cpp文件夹中)
1. 加权限 chmod +x buildbit.sh
2. 生成Debug版bitctrl  ./buildbit.sh Debug
3. 生成Release版bitctrl  ./buildbit.sh Release
3. 在要使用bitctrl的node工程中，修改CMakeLists.txt，加入bitctrl的依赖
3.1 设置bitctrl安装目录   即 set(bitctrl_DIR /home/waterdrop/code/cpp/install-all/lib/cmake/bitctrl)
3.2 让CMake找bitctrl     即 find_package(bitctrl REQUIRED)
3.3 让node链接bitctrl    即 target_link_libraries(node名字 bitctrl)
注释：对于给的示例工程bitrobot，只需要如3.1配置bitctrl_DIR为运行buildbit.sh后生成的目录

(catkin文件夹中)
4. bitctrl的使用参见提供的bitrobot示例
5. 注意，由于bitctrl的初始化需要读取配置文件simpleConfig.txt，所以需要设置好配置文件所在目录
   即修改bitrobot_node.cpp中第136行:  myconfig.config_dir=""
   保证该目录下有simpleConfig.txt这一文件（simpleConfig.txt见config文件夹）
   
## How to Run (bitform)
1. 运行stage, 如在catkin_ws下运行: 
   rosrun stage_ros stageros /home/waterdrop/code/testbit.world
   说明: testbit.world里定义了4个机器人
2. 运行程序bitform
   roslaunch bitrobot form4.launch
2. 发送编队前进任务 (topic=formation_goal, type=geometry_msgs/PoseStamped):
   rostopic pub formation_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -5.5, y: 1, z: 0.0}, orientation: {w: 1.0}}}'






