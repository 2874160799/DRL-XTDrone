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


# 检查目标点是否在障碍物上
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


class Loop:
    def __init__(self):
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
            # 设置箭头和朝向
        self.marker.pose = self.goal_pose.pose

        # 设置箭头颜色和大小
        self.marker.scale.x = 0.5  # 箭头长度
        self.marker.scale.y = 0.1  # 箭头宽度
        self.marker.scale.z = 0.1  # 箭头厚度
        self.marker.color.r = 0.0  # 红色
        self.marker.color.g = 0.0  # 无绿色
        self.marker.color.b = 1.0  # 蓝色
        self.marker.color.a = 1.0  # 完全不透明
        
        self.loop_flag = True
        self.loop_times = 1
        
        #publisher
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        #subscriber
        self.odom_sub = rospy.Subscriber("/iris_0/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=10)

     
     
     
     
     
     
       
    def odom_callback(self,msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y


    #距离目标点半径1m即判断到达目标点
    def is_within_radius(self,target_x, target_y, radius):
        distance = math.sqrt((self.drone_x - target_x)**2 + (self.drone_y - target_y)**2)
        return distance <= radius

    #检查模型是否存在
    def check_model_exists(self, model_name):
    # 创建服务客户端
        try:
            client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            srv = gazebo_msgs.srv.GetWorldPropertiesRequest()
            
            # 调用服务并检查返回结果
            response = client.call(srv)
            
            # 检查模型名称是否存在
            for name in response.model_names:
                if name == model_name:
                    return True  # 模型存在
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        return False  # 模型不存在    
    
    #删除模型    
    # def delete_model_in_gazebo(self, model_name):
    #     try:
    #         # 创建服务客户端
    #         client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    #         delete_srv = DeleteModel()
    #         delete_srv.model_name = model_name
            
    #         # 调用服务并检查返回结果
    #         response = client(delete_srv)
            
    #         if response.success:
    #             rospy.loginfo(f"Model [{model_name}] deleted successfully!")
    #             return True
    #         else:
    #             rospy.logwarn(f"Failed to delete model [{model_name}]: {response.status_message}")
    #             return False
    #     except rospy.ServiceException as e:
    #         rospy.logwarn(f"Service call failed: {e}")
    #         return False
    def delete_gazebo_model(model_name):
        """
        删除Gazebo中的模型。

        参数:
        model_name (str): 要删除的模型的名称。
        """
        # 创建服务客户端，连接到 Gazebo 的 delete_model 服务
        try:
            delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            rospy.wait_for_service('/gazebo/delete_model')  # 等待服务可用
        except rospy.ROSException as e:
            rospy.logerr("Service /gazebo/delete_model not available: %s", str(e))
            return False

        # 调用服务，删除模型
        try:
            response = delete_model_client(model_name)
            if response.success:
                rospy.loginfo("Successfully deleted model: %s", model_name)
                return True
            else:
                rospy.logerr("Failed to delete model: %s", model_name)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return False
        
    def send_goal(self, x, y):
        rospy.loginfo("THIS IS SEND_GOAL_NODE")
        
        # 创建move_base的action客户端
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for move_base action server ...")
        ac.wait_for_server()
        
        # 创建目标消息
        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置目标点的位置和朝向
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # 无旋转，保持朝向
        
        # 发布目标点
        rospy.loginfo("sending goal to move_base ...")
        ac.send_goal(goal)
        
        # 等待机器人到达目标点
        #rate = rospy.Rate(10)  # 10hz
        #while not rospy.is_shutdown():
        while True:
            # rospy.spin()
            # rate.sleep()
            if self.is_within_radius(x, y, 1.0):
                rospy.loginfo("SUCCESSFULLY reached the goal!")
                ac.cancel_goal()
                break
            if ac.get_state() == GoalStatus.ABORTED:
                rospy.loginfo("failed to reach the goal")
                break
    
    def spawn_model_in_gazebo(self, model_name, model_file, pose):
    # 创建服务客户端，连接到 Gazebo 的 spawn_model 服务
        try:
            spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            rospy.wait_for_service('/gazebo/spawn_sdf_model')  # 等待服务可用
        except rospy.ROSException as e:
            rospy.logerr("Service /gazebo/spawn_sdf_model not available: %s", str(e))
            return False

        # 读取模型文件
        try:
            with open(model_file, 'r') as file:
                model_xml = file.read()
        except FileNotFoundError:
            rospy.logerr("Model file not found: %s", model_file)
            return False

        # 创建 SpawnModel 服务请求
        spawn_srv = SpawnModel()
        spawn_srv.model_name = model_name
        spawn_srv.model_xml = model_xml
        spawn_srv.robot_namespace = ""
        spawn_srv.initial_pose = pose
        spawn_srv.reference_frame = "world" 

        # 调用服务
        try:
            response = spawn_model_client(model_name, model_xml, "", pose, "world")
            if response.success:
                rospy.loginfo("Successfully spawned model: %s", model_name)
                return True
            else:
                rospy.logerr("Failed to spawn model: %s", model_name)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return False
        
    
    def spawn_model_thread(self, x, y, i):
        # 设置模型名称、路径和位置
        model_name = "arrow_red_" + str(i)
        old_model_name = "arrow_red_" + str(i - 1)
        model_file = "/home/ubuntu/.gazebo/models/arrow_red/model.sdf"
        
        # 设置模型的位置
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 5.0

        # 设置四元数，绕y轴旋转90度
        pose.orientation.x = -sin(pi / 4)
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = cos(pi / 4)

        # 检查上一个模型是否已经存在，存在则删除
        if self.check_model_exists(old_model_name):
            rospy.loginfo("Model [%s] already exists, deleting it first.", old_model_name)
            self.delete_model_in_gazebo(old_model_name)
            if not self.delete_model_in_gazebo(old_model_name):
                rospy.logerr("Failed to delete existing model [%s], cannot proceed with spawning.", old_model_name)
                return

        # 放置新的模型
        if self.spawn_model_in_gazebo(model_name, model_file, pose):
            rospy.loginfo("Model spawned successfully!")
            return
        else:
            rospy.logerr("Failed to spawn model!")
            return
            
                
            
            
    def run(self):
        while self.loop_flag:
            while True:
                x = random.uniform(-10,10)
                y = random.uniform(-12,8)
                
                if check_goal(x,y):
                    rospy.loginfo("The goal coordinate is (%.1f, %.1f)",x,y)
                    break
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            self.marker.pose = goal_pose.pose
            
            #1HZ频率发布标记
            rospy.Timer(rospy.Duration(1.0),lambda event:self.marker_pub.publish(self.marker))
            
            goal_thread = threading.Thread(target=self.send_goal, args=(x, y))
            #spawn_thread = threading.Thread(target=self.spawn_model_thread, args=(x, y, self.loop_times))
            
            goal_thread.start()
            #spawn_thread.start()
            goal_thread.join()
            #spawn_thread.join()
            
            rospy.spin()
            sleep(0.1)
            self.loop_times += 1
            