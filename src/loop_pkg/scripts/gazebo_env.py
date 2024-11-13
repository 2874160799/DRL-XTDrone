#!/usr/bin/env python3
import rospy
import random
import math
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
from geometry_msgs.msg import Pose,PoseStamped
from nav_msgs.msg import Odometry
import threading
from std_msgs.msg import Header
from math import pi, sin, cos
from time import sleep
import gazebo_msgs
from actionlib_msgs.msg import GoalStatus

update_flag = False

def check_goal(x, y):
    goal_ok = False
    # 1
    if -7 > x and x > -9 and 7 > y and y > -1:
        goal_ok = True
    # 2
    if -2 > x and x > -5 and 7 > y and y > 2.5:
        goal_ok = True
    # 3
    if -1.5 > x and x > -5 and 0.5 > y and y > -1:
        goal_ok = True
    # 10
    if -5 > x and x > -9 and -2.5 > y and y > -7.5:
        goal_ok = True
    # 11
    if -5 > x and x > -9 and -9.5 > y and y > -11:
        goal_ok = True
    # 4
    if 2 > x and x > 0.5 and 7 > y and y > -11:
        goal_ok = True
    # 5
    if 9 > x and x > 4 and 7 > y and y > 5:
        goal_ok = True
    # 6
    if 5.5 > x and x > 4 and 2.5 > y and y > -6.5:
        goal_ok = True
    # 7
    if 9 > x and x > 7.5 and 2.5 > y and y > -6.5:
        goal_ok = True
    # 8
    if 9 > x and x > 3 and -8.5 > y and y > -11:
        goal_ok = True
    # 9
    if -1.5 > x and x > -2.5 and -3 > y and y > -11:
        goal_ok = True

    return goal_ok

def random_coordinate():
    global random_x,random_y
    random_x = random.uniform(-10,10)
    random_y = random.uniform(-12,8)
    if check_goal(random_x,random_y):
        rospy.loginfo("The goal coordinate is (%.1f, %.1f)",random_x,random_y)
        
        # self.pose.position.x = random_x
        # self.pose.position.y = random_y

class GazeboModel:
    def __init__(self):
        rospy.init_node('gazebo_model_manager', anonymous=True)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        rospy.wait_for_service('/gazebo/get_world_properties')
        self.spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        
        self.model_name = "arrow_red_"
        self.model_file = "/home/ubuntu/.gazebo/models/arrow_red/model.sdf"
        
        global random_x,random_y
        random_x = 0.0
        random_y = 0.0
        self.pose = Pose()
        self.pose.position.x = random_x
        self.pose.position.y = random_y
        self.pose.position.z = 5.0
        self.pose.orientation.x = -sin(pi / 4)
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = cos(pi / 4)
        
        self.loop_times = 1
        
        with open(self.model_file, 'r') as file:
                self.model_xml = file.read()
                
    def spawn_model(self, model_name, model_xml, pose, namespace=""):
        """
        创建模型到Gazebo环境中
        :param model_name: 模型名称
        :param model_xml: 模型的SDF或URDF字符串
        :param pose: geometry_msgs/Pose类型 模型的初始位姿
        :param namespace: 模型的命名空间
        """
        try:
            response = self.spawn_model_client(model_name, model_xml, namespace, pose, "world")
            if response.success:
                rospy.loginfo(f"Model {model_name} spawned successfully.")
            else:
                rospy.logwarn(f"Failed to spawn model {model_name}: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def delete_model(self, model_name):
        """
        从Gazebo环境中删除模型
        :param model_name: 模型名称
        """
        try:
            response = self.delete_model_client(model_name)
            if response.success:
                rospy.loginfo(f"Model {model_name} deleted successfully.")
            else:
                rospy.logwarn(f"Failed to delete model {model_name}: {response.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    

    
    def check_model_exists(self, model_name):
        try:
            response = self.get_world_properties()
            if model_name in response.model_names:
                return True
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")  
            return False         
    
    def UpdateGazeboModel(self,loop_times):
        old_model_name = self.model_name + str(loop_times - 1)   
        model_name =self.model_name + str(loop_times)  
        
        if self.check_model_exists(old_model_name):
            self.delete_model(old_model_name)
            rospy.loginfo("model exists,delete first")
            sleep(1)
            self.spawn_model(model_name,self.model_xml,self.pose)
        else:
            self.spawn_model(model_name,self.model_xml,self.pose)
            
class Goal:
    def __init__(self):
        #rviz
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.goal_pose = PoseStamped()
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "goal_arrow"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        
        self.marker.scale.x = 0.5  # 箭头长度
        self.marker.scale.y = 0.1  # 箭头宽度
        self.marker.scale.z = 0.1  # 箭头厚度
        self.marker.color.r = 0.0  # 红色
        self.marker.color.g = 0.0  # 无绿色
        self.marker.color.b = 1.0  # 蓝色
        self.marker.color.a = 1.0  # 完全不透明
        
        global random_x,random_y
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = random_x
        self.goal_pose.pose.position.y = random_y
        self.goal_pose.pose.orientation.w = 1.0
        self.marker.pose = self.goal_pose.pose
        
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber("/iris_0/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=10)
        
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        
    
    def odom_callback(self,msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
       
        #距离目标点半径1m即判断到达目标点
    def is_within_radius(self,target_x, target_y, radius):
        distance = math.sqrt((self.drone_x - target_x)**2 + (self.drone_y - target_y)**2)
        return distance <= radius
    
    def send_goal(self,x,y):
        self.ac.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
            # 设置目标点的位置和朝向
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # 无旋转，保持朝向
     
        rospy.loginfo("sending goal to move_base ...")
        self.ac.send_goal(goal)
        while not rospy.is_shutdown():
            if self.is_within_radius(x, y, 1.0):
                rospy.loginfo("SUCCESSFULLY reached the goal!")
                self.ac.cancel_goal()
                global update_flag 
                update_flag = True
                break
            if self.ac.get_state() == GoalStatus.ABORTED:
                rospy.loginfo("failed to reach the goal")
                break
        

if __name__ == '__main__':
    arrow_red = GazeboModel()
    goal = Goal()
    while not rospy.is_shutdown():
        random_coordinate()
        arrow_red.pose.position.x = random_x
        arrow_red.pose.position.y = random_y
        goal.goal_pose.pose.position.x = random_x
        goal.goal_pose.pose.position.y = random_y
        
        rospy.Timer(rospy.Duration(1.0),lambda event:goal.marker_pub.publish(goal.marker))
        
        arrow_red.UpdateGazeboModel(arrow_red.loop_times)
        goal.send_goal(random_x,random_y)
        arrow_red.loop_times +=1