#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将发布/person_info话题，自定义消息类型learning_topic::Person

import rospy
# from learning_topic.msg import Person
from geometry_msgs.msg import Twist

control_speed=-1
control_turn=1


def velocity_publisher():
    # ROS节点初始化
    rospy.init_node('velocity_publisher', anonymous=True)

    # 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    print("publisher")
    #设置循环的频率
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        twist=Twist()
        twist.linear.x = -1     
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 2
        pub.publish(twist)
        # 按照循环频率延时
        rate.sleep()

    # while not rospy.is_shutdown():
    # 	twist=Twist()
    # 	twist.linear.x = control_speed; 
    # 	twist.linear.y = 0; 
    # 	twist.linear.z = 0
    # 	twist.angular.x = 0; 
    # 	twist.angular.y = 0; 
    # 	twist.angular.z = control_turn
    # 	pub.publish(twist)

    # 	# 发布消息
    # 	# person_info_pub.publish(person_msg)
    # 	# rospy.loginfo("Publsh person message[%s, %d, %d]", 
    # 	# 		person_msg.name, person_msg.age, person_msg.sex)

    # 	# 按照循环频率延时
    # 	rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
        
    except rospy.ROSInterruptException:
        pass
