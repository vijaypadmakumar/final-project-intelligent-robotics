#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
    rospy.init_node("Mover", anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = 0.1
        base_data.angular.z = 0.1
        printPoint(base_data)
        
        # printPoint(base_data)
        pub.publish(base_data)
        # rate.sleep()

def printPoint(msg):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "/map"
    point.point.x = msg.point.x
    point.point.y = msg.point.y
    point.point.z = msg.point.z
    rospy.loginfo("coordinates:x=%f y=%f" %(point.point.x, point.point.y))

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
