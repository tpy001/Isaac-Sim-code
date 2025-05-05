#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")  # 处理接收的消息

def listener():
    rospy.init_node('listener', anonymous=True)  # 初始化 ROS 节点
    rospy.Subscriber('chatter', String, callback)  # 订阅 "chatter" 话题
    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    listener()
