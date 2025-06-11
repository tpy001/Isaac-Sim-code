#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)  # 初始化 ROS 节点
    pub = rospy.Publisher('chatter', String, queue_size=10)  # 定义发布者
    rate = rospy.Rate(1)  # 1Hz 发送消息

    while not rospy.is_shutdown():
        message = "Hello from Process A"
        rospy.loginfo(f"Publishing: {message}")
        pub.publish(message)  # 发送消息
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
