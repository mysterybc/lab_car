#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
import tf
from std_msgs.msg import String
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  

class NavTest():  
    def __init__(self):  
        rospy.init_node('robot2_navigation', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        self.state=0
        self.plan = 0
        self.door_plan = 0
        self.open_state = 0

        self.tf_listener = tf.TransformListener()
 
        # 在每个目标位置暂停的时间  
        self.rest_time = rospy.get_param("~rest_time", 0.01)  


        self.sub = rospy.Subscriber('door_state', String, self.callback)

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  
        # 如果想要获得某一点的坐标，在rviz中点击 2D Nav Goal 按键，然后单机地图中一点  
        # 在终端中就会看到坐标信息  
        self.locations = dict()  

        self.locations['p1'] = Pose(Point(6.000, 0.000, 0.000), Quaternion(0.000, 0.000, 0.707, 0.707))  
        self.locations['p4'] = Pose(Point(5.000,3.000, 0.000), Quaternion(0.000, 0.000, 1.000, 0.000))  
        self.locations['p2'] = Pose(Point(3.500, 1.000, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))  
        self.locations['p3'] = Pose(Point(5.000, -3.5000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
        
        
        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=5)  
 
        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("/robot2/move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        self.n_locations = len(self.locations)  
        self.n_goals = 0  
        self.n_successes = 0  
        self.i = self.n_locations  
         
        start_time = rospy.Time.now()  
        running_time = 0  
        self.location = ""  
        self.last_location = "" 


        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")  

        


    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

    def circle(self):
        
        # 如果已经走完了所有点，再重新开始排序  
        if self.i == self.n_locations:  
            self.i = 0  
            #sequence = sample(locations, n_locations)  
            #print(sequence)
            self.sequence=list(self.locations)
            print(self.sequence)
            # 如果最后一个点和第一个点相同，则跳过  
            if self.sequence[0] == self.last_location:  
                self.i = 1  

        # 在当前的排序中获取下一个目标点  
        location = self.sequence[self.i]  


      
 

        # 设定下一个目标点  
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.pose = self.locations[location]  
        self.goal.target_pose.header.frame_id = '/map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()  

         
       
        
        if self.plan == 0 :
            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)
             # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(location))   

            self.plan=1  



       
        state = self.move_base.get_state()  
        if state == GoalStatus.SUCCEEDED:  
            rospy.loginfo("Goal succeeded!")  
            self.n_successes += 1  
            self.plan=0   
            self.i += 1  
            self.n_goals += 1 

            rospy.loginfo("State:" + str(state))  

    def open_door(self):
        goal_pose=Pose(Point(8.000, 3.000, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))  

        self.goal = MoveBaseGoal() 
        self.goal.target_pose.pose = goal_pose 
        self.goal.target_pose.header.frame_id = '/map'  
        self.goal.target_pose.header.stamp = rospy.Time.now()

        

        if self.open_state == 0 and self.door_plan == 0:
            self.move_base.send_goal(self.goal) 
            self.door_plan=1
            rospy.loginfo("plan door")
         # 五分钟时间限制  
        
        state = self.move_base.get_state() 
        if state == GoalStatus.SUCCEEDED:  
             
            
            self.open_state=1
            self.door_plan=0
            rospy.loginfo("open door")

    def callback(self,data):

         if data.data == 'open':
            self.state=1
           
            
         elif data.data == '2':
            self.state=0
            self.open_state=0
            

    def get_pos(self): 
        try: 
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/robot2/base_link', rospy.Time(0)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            rospy.loginfo("tf Error") 
            return None 
        
        
        
        x = trans[0] 
        y = trans[1] 
        
        return (x, y)
        

def trunc(f, n):   
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  




if __name__ == '__main__':  
    try:  
        robot1=NavTest()  
        while not rospy.is_shutdown():
            if robot1.state == 1:
                robot1.open_door()
            else:
                robot1.circle()

            
        
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Random navigation finished.")
