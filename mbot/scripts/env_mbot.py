#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person

import re
# from urllib import response
import rospy
from nav_msgs.msg._Odometry import Odometry
from gazebo_msgs.srv._SpawnModel import SpawnModelRequest
from gazebo_msgs.srv._SetModelState import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import math
import numpy as np

# flag=1

# def positionInfoCallback(msg):
    
#     # rospy.loginfo("Subcribe Person Info: x:%s  age:%d  sex:%d", 
# 	# 		 msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
#     print("Subcribe Position Info: x:%s  y:%s  o:%s"%
# 			 (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z))
#     print("Subcribe Velocity Info: x:%s  y:%s  w:%s "%
# 			 (msg.twist.twist.linear.x, msg.twist.twist.linear.y,msg.twist.twist.angular.z))
#     flag=1

# def person_subscriber():
# 	# ROS节点初始化
#     rospy.init_node('person_subscriber', anonymous=True)

# 	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
#     rospy.Subscriber("/odom", Odometry, positionInfoCallback)
    

#     if (flag==1):
#         # 循环等待回调函数
#         rospy.spin()
#     else:
#         rospy.wait_for_service('/gazebo/set_model_state')
#         reset  = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
#         state = ModelState()
#         for i in range(len(self.gazebo_model_states.name)):
#             if self.gazebo_model_states.name[i] == "point_start":
#                 state.reference_frame = 'world'
#                 state.pose.position.z = 0.0
#                 state.model_name = self.gazebo_model_states.name[i]
#                 state.pose.position.x = self.sp[0]
#                 state.pose.position.y = self.sp[1]
#                 val(state)
#             if self.gazebo_model_states.name[i] == "point_goal":
#                 state.reference_frame = 'world'
#                 state.pose.position.z = 0.0
#                 state.model_name = self.gazebo_model_states.name[i]
#                 state.pose.position.x = self.gp[0]
#                 state.pose.position.y = self.gp[1]
#                 val(state)
#             if self.gazebo_model_states.name[i] == self.agentrobot:
#                 state.reference_frame = 'world'
#                 state.pose.position.z = 0.0
#                 state.model_name = self.gazebo_model_states.name[i]
#                 rpy = [0.0, 0.0, randangle]
#                 q = self.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
#                 state.pose.orientation.x = q[0]
#                 state.pose.orientation.y = q[1]
#                 state.pose.orientation.z = q[2]
#                 state.pose.orientation.w = q[3]
#                 state.pose.position.x = self.sp[0]
#                 state.pose.position.y = self.sp[1]
#                 val(state)
#                 # 到目标点的距离
#                 self.d = math.sqrt((state.pose.position.x - self.gp[0])**2 + (state.pose.position.y - self.gp[1])**2)
#         try:
#             reset = rospy.ServiceProxy('/gazebo/reset_simulation', Spawn)

#             # 请求服务调用，输入请求数据
#             response1 = reset()

#             model_add=rospy.ServiceProxy('/gazebo/spawn_urdf_model',SpawnModelRequest)
#             response2 = model_add()

            
#         except rospy.ServiceException:
#             print ("Service call failed: %s"%"e")


MAXENVSIZE = 30.0  # 边长为30的正方形作为环境的大小
Image_matrix = []

# if __name__ == '__main__':
#     person_subscriber()

class envmodel():
    
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        '''
        # 保存每次生成的map信息
        self.count_map = 1
        self.foldername_map='map'
        if os.path.exists(self.foldername_map):
            shutil.rmtree(self.foldername_map)
        os.mkdir(self.foldername_map)
        '''

        # agent列表
        self.agentrobot = 'agent'
        
        self.img_size = 80

        # 障碍数量
        self.num_obs = 0

        self.dis = 1.0  # 位置精度-->判断是否到达目标的距离

        self.obs_pos = []  # 障碍物的位置信息

        self.gazebo_model_states = ModelStates()
        
        self.bridge       = CvBridge()
        self.image_matrix = []
        self.image_matrix_callback = []

        self.resetval()

        # 接收gazebo的modelstate消息
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_states_callback)
        
        # 接收agent robot的前视bumblebee相机消息
        self.subimage = rospy.Subscriber('/ns0/image_raw', Image, self.image_callback)
        # # 接收agent robot的激光雷达信息
        # self.subLaser = rospy.Subscriber('/' + self.agentrobot +'/front/scan', LaserScan, self.laser_states_callback)
        # 发布控制指令给agent robot
        self.pub = rospy.Publisher('/ns0/cmd_vel', Twist, queue_size=10)
        
        time.sleep(1.0)

    def resetval(self):
        self.robotstate = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,v,w,yaw,vx,vy
        self.d          = 0.0                                  # 到目标的距离
        self.d_last     = 0.0                                  # 前一时刻到目标的距离
        self.v_last     = 0.0                                  # 前一时刻的速度
        self.w_last     = 0.0                                  # 前一时刻的角速度
        self.r          = 0.0                                  # 奖励
        self.cmd        = [0.0, 0.0]                           # agent robot的控制指令
        self.done_list  = False                                # episode是否结束的标志

    def gazebo_states_callback(self, data):
        self.gazebo_model_states = data
        # name: ['ground_plane', 'jackal1', 'jackal2', 'jackal0',...]
        for i in range(len(data.name)):
            if data.name[i] == self.agentrobot:
                # robotstate--->x,y,v,w,yaw,vx,vy
                self.robotstate[0] = data.pose[i].position.x
                self.robotstate[1] = data.pose[i].position.y
                v = math.sqrt(data.twist[i].linear.x**2 + data.twist[i].linear.y**2)
                self.robotstate[2] = v
                self.robotstate[3] = data.twist[i].angular.z
                rpy = self.euler_from_quaternion(data.pose[i].orientation.x,data.pose[i].orientation.y,
                data.pose[i].orientation.z,data.pose[i].orientation.w)
                self.robotstate[4] = rpy[2]
                self.robotstate[5] = data.twist[i].linear.x
                self.robotstate[6] = data.twist[i].linear.y

    def image_callback(self, data):
        try:
            self.image_matrix_callback = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            cv2.imshow("frame" , self.image_matrix_callback)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def quaternion_from_euler(self, r, p, y):
        q = [0, 0, 0, 0]
        q[3] = math.cos(r / 2) * math.cos(p / 2) * math.cos(y / 2) + math.sin(r / 2) * math.sin(p / 2) * math.sin(y / 2)
        q[0] = math.sin(r / 2) * math.cos(p / 2) * math.cos(y / 2) - math.cos(r / 2) * math.sin(p / 2) * math.sin(y / 2)
        q[1] = math.cos(r / 2) * math.sin(p / 2) * math.cos(y / 2) + math.sin(r / 2) * math.cos(p / 2) * math.sin(y / 2)
        q[2] = math.cos(r / 2) * math.cos(p / 2) * math.sin(y / 2) - math.sin(r / 2) * math.sin(p / 2) * math.cos(y / 2)
        return q

    def euler_from_quaternion(self, x, y, z, w):
        euler = [0, 0, 0]
        Epsilon = 0.0009765625
        Threshold = 0.5 - Epsilon
        TEST = w * y - x * z
        if TEST < -Threshold or TEST > Threshold:
            if TEST > 0:
                sign = 1
            elif TEST < 0:
                sign = -1
            euler[2] = -2 * sign * math.atan2(x, w)
            euler[1] = sign * (math.pi / 2.0)
            euler[0] = 0
        else:
            euler[0] = math.atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
            euler[1] = math.asin(-2 * (x * z - w * y))
            euler[2] = math.atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
        
        return euler

    def getreward(self):
        
        reward = 0

        # 假如上一时刻到目标的距离<这一时刻到目标的距离就会有负的奖励
        if self.d_last < self.d:
            reward = reward - 0.1*(self.d-self.d_last)
        
        if self.d_last >= self.d:
            reward = reward + 0.1*(self.d_last - self.d)
        
        # 速度发生变化就会有负的奖励
        reward = reward - 0.01*(abs(self.w_last - self.cmd[1]) + abs(self.v_last - self.cmd[0])) 
        
        # 到达目标点有正的奖励
        # if self.d < self.dis and not self.done_list:
        if self.d < self.dis:
            reward = reward + 20
            print("Get 20 reward------goal point!!!!!!")
            # self.done_list = True
        '''
        # 碰撞障碍物有负的奖励
        for i in range(len(self.obs_pos)):
            if math.sqrt((self.robotstate[0]-self.obs_pos[i][0])**2 + (self.robotstate[1]-self.obs_pos[i][1])**2) < 1:
                reward = reward - 1
                self.done_list = True
        '''
        return reward


    # 重置environment
    def reset_env(self, start=[0.0, 0.0], goal=[10.0, 10.0], Randangle=0.0):
        self.sp = start
        self.gp = goal
        # 初始点到目标点的距离
        self.d_sg = ((self.sp[0]-self.gp[0])**2 + (self.sp[1]-self.gp[1])**2)**0.5
        # 重新初始化各参数
        self.resetval()
        '''
        #获取障碍的随机初始位置
        #self.obs_pos = self.random_square(MAXENVSIZE/2)
        #障碍物的半径是0.5m
        while(True):
            self.obs_pos = self.random_square(MAXENVSIZE/2)
            flag = True  # 用于判断起点和终点是否在障碍物的范围内
            for i in range(self.num_obs):
                # 如果起点在障碍物周围1.5m的范围内则需要重新生成障碍物
                if math.sqrt((self.sp[0]-self.obs_pos[i][0])**2+(self.sp[1]-self.obs_pos[i][1])**2) < 2:
                    flag = False
                    # print("obtacle generating fail for start point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                # 如果终点在障碍物周围1.5m的范围内则需要重新生成障碍物
                if math.sqrt((self.gp[0]-self.obs_pos[i][0])**2+(self.gp[1]-self.obs_pos[i][1])**2) < 2:
                    flag = False
                    # print("obtacle generating fail for state for goal point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                # 如果两个障碍相隔太近则需要重新生成障碍物
                for j in range(i + 1, self.num_obs):
                    if math.sqrt((self.obs_pos[i][0]-self.obs_pos[j][0])**2+(self.obs_pos[i][1]-self.obs_pos[j][1])**2) < 2:
                        flag = False
                        # print("obtacle generating fail for obstacle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")     
            if flag == True:
                break
        '''
        rospy.wait_for_service('/gazebo/set_model_state')
        val = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        randomposition = 2 * self.dis * np.random.random_sample((1, 2)) - self.dis
        # agent robot生成一个随机的角度
        randangle=Randangle
        # randangle = 2 * math.pi * np.random.random_sample(1) - math.pi
        # 根据model name对每个物体的位置初始化
        state = ModelState()
        for i in range(len(self.gazebo_model_states.name)):
            # if self.gazebo_model_states.name[i] == "point_start":
            #     state.reference_frame = 'world'
            #     state.pose.position.z = 0.0
            #     state.model_name = self.gazebo_model_states.name[i]
            #     state.pose.position.x = self.sp[0]
            #     state.pose.position.y = self.sp[1]
            #     val(state)
            # if self.gazebo_model_states.name[i] == "point_goal":
            #     state.reference_frame = 'world'
            #     state.pose.position.z = 0.0
            #     state.model_name = self.gazebo_model_states.name[i]
            #     state.pose.position.x = self.gp[0]
            #     state.pose.position.y = self.gp[1]
            #     val(state)
            if self.gazebo_model_states.name[i] == "goal":
                state.reference_frame = 'world'
                state.pose.position.z = 0.0
                state.model_name = self.gazebo_model_states.name[i]
                state.pose.position.x = self.gp[0]
                state.pose.position.y = self.gp[1]
                val(state)
            if self.gazebo_model_states.name[i] == self.agentrobot:
                state.reference_frame = 'world'
                state.pose.position.z = 0.0
                state.model_name = self.gazebo_model_states.name[i]
                rpy = [0.0, 0.0, randangle]
                q = self.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                state.pose.orientation.x = q[0]
                state.pose.orientation.y = q[1]
                state.pose.orientation.z = q[2]
                state.pose.orientation.w = q[3]
                state.pose.position.x = self.sp[0]
                state.pose.position.y = self.sp[1]
                #发送服务
                val(state)
                # 到目标点的距离
                self.d = math.sqrt((state.pose.position.x - self.gp[0])**2 + (state.pose.position.y - self.gp[1])**2)
            '''
            for k in range(self.num_obs):
                NAME_OBS = 'obs' + str(k)
                if self.gazebo_model_states.name[i] == NAME_OBS:
                    state.reference_frame = 'world'
                    state.pose.position.z = 0.0
                    state.model_name = self.gazebo_model_states.name[i]
                    state.pose.position.x = self.obs_pos[k][0]
                    state.pose.position.y = self.obs_pos[k][1]            
                    val(state)
            '''
        self.done_list = False  # episode结束的标志
        time.sleep(2.0)

    def get_env(self):
        env_info=[]
        observation=[]
        # input2-->agent robot的v,w,d,theta
        selfstate = [0.0, 0.0, 0.0, 0.0]
        # robotstate--->x,y,v,w,yaw,vx,vy
        selfstate[0] = self.robotstate[2]  # v
        selfstate[1] = self.robotstate[3]  # w
        # d代表agent机器人距离目标的位置-->归一化[0,1]
        selfstate[2] = self.d/MAXENVSIZE
        dx = -(self.robotstate[0]-self.gp[0])
        dy = -(self.robotstate[1]-self.gp[1])
        xp = dx*math.cos(self.robotstate[4]) + dy*math.sin(self.robotstate[4])
        yp = -dx*math.sin(self.robotstate[4]) + dy*math.cos(self.robotstate[4])
        thet = math.atan2(yp, xp)
        selfstate[3] = thet/math.pi

        # #######################################input1-->雷达信息#########################################3333
        # laser = []
        # temp = []
        # sensor_info = []
        # for j in range(len(self.laser.ranges)):
        #     tempval = self.laser.ranges[j]
        #     # 归一化处理
        #     if tempval > MAXLASERDIS:
        #         tempval = MAXLASERDIS
        #     temp.append(tempval/MAXLASERDIS)
        # laser = temp
        # # 将agent robot的input2和input1合并成为一个vector:[input2 input1]

        # # env_info.append(laser)
        # # env_info.append(selfstate)
        # for i in range(len(laser)+len(selfstate)):
        #     if i<len(laser):
        #         sensor_info.append(laser[i])
        #     else:
        #         sensor_info.append(selfstate[i-len(laser)])
        # #第一纬度是相机信息
        # # input1-->相机
        # env_info.append(sensor_info)
        # #print("The state is:{}".format(state))

        
        # shape of image_matrix [768,1024,3]
        # self.image_matrix = np.uint8(self.image_matrix_callback)
        # self.image_matrix = cv2.resize(self.image_matrix, (self.img_size, self.img_size))
        # # shape of image_matrix [80,80,3]
        # self.image_matrix = cv2.cvtColor(self.image_matrix, cv2.COLOR_RGB2GRAY)
        # # shape of image_matrix [80,80]
        # self.image_matrix = np.reshape(self.image_matrix, (self.img_size, self.img_size))
        # # shape of image_matrix [80,80]
        # cv2.imshow("Image window", self.image_matrix)
        # cv2.waitKey(2)
        # (rows,cols,channels) = self.image_matrix.shape
        # print("image matrix rows:{}".format(rows))
        # print("image matrix cols:{}".format(cols))
        # print("image matrix channels:{}".format(channels))
        env_info.append("self.image_matrix")
        # print("shape of image matrix={}".format(self.image_matrix.shape))

        # 判断是否终止
        self.done_list = True
        # 是否到达目标点判断
        if self.d > self.dis:
            self.done_list = False  # 不终止
        else:
            self.done_list = True  # 终止
            print("Goal Point!")
        '''
        # 障碍物判断
        for i in range(len(self.obs_pos)):
            # 障碍物的半径为0.5m
            if math.sqrt((self.robotstate[0]-self.obs_pos[i][0])**2 + (self.robotstate[1]-self.obs_pos[i][1])**2) >= 1.0:
                self.done_list = False  # 不终止
        '''
        env_info.append(self.done_list)

        self.r = self.getreward()
        
        env_info.append(self.r)
        
        # 小车的坐标
        jackal_x = self.robotstate[0]
        jackal_y = self.robotstate[1]

        self.v_last = self.cmd[0]
        self.w_last = self.cmd[1]
        observation.append(jackal_x)
        observation.append(jackal_y)
        observation.append(self.gp[0])
        observation.append(self.gp[1])
        observation.append(self.v_last)
        observation.append(self.w_last)
        observation=np.array(observation)
        return observation,self.r,self.done_list
    

    def step(self, cmd=[1.0, 0.0]):
        self.d_last = math.sqrt((self.robotstate[0] - self.gp[0])**2 + (self.robotstate[1] - self.gp[1])**2)
        self.cmd[0] = cmd[0]
        self.cmd[1] = cmd[1]
        cmd_vel = Twist()
        cmd_vel.linear.x  = cmd[0]
        cmd_vel.angular.z = cmd[1]
        self.pub.publish(cmd_vel)
        
        time.sleep(0.05)

        self.d = math.sqrt((self.robotstate[0] - self.gp[0])**2 + (self.robotstate[1] - self.gp[1])**2)
        self.v_last = cmd[0]
        self.w_last = cmd[1]



if __name__ == '__main__':
    env=envmodel()
    
    env.reset_env(start=[5.0, 5.0], goal=[-5.0,-5.0])
    del env
    pass