#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def mapmove():
    pub = rospy.Publisher("map_move", Twist, queue_size=100)
    rospy.init_node("Map_Mover", anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        base_data = Twist()
        base_data.linear.x = 0.1
        
        # printPoint(base_data)
        pub.publish(base_data)
        # rate.sleep()

if __name__ == "__main__":
    try:
        mapmove()
    except rospy.ROSInterruptException:
        pass