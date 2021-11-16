#!/usr/bin/env python3
# from re import X
import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# laser_range = []

# def get_direction_averages(ranges):
#     left = ranges[:167]
#     center = ranges[167:333]
#     right = ranges[333:]
#     left_avg = sum(left)/len(left)
#     center_avg = sum(center)/len(center)
#     right_avg = sum(right)/len(right)

#     return left_avg, center_avg, right_avg

# def findDirection(ranges):
#     left_avg, center_avg, right_avg = get_direction_averages(ranges)
#     # if left_avg >= center_avg:
#     #     if left_avg >= right_avg:
#     #         return "LEFT"
#     #     else:
#     #         steer = "right"
#     # elif center_avg >= right_avg:
#     #     steer = "center"
#     # elif right_avg > center_avg:
#     #     steer = "right"

#     max_avg = max(left_avg, center_avg, right_avg)

#     if max_avg == left_avg:
#         return "l"
#     elif max_avg == center_avg:
#         return "c"
#     else:
#         return "r"


# def callback(msg):
#     print(len(msg.ranges))
#     direction = findDirection(msg.ranges)
#     print(direction)
#     filewritter = open("direction.txt", "w")
#     filewritter.write(direction)
#     filewritter.close()

    # file = open("log.txt", "a")
    # file.write(f"{msg.ranges} \"\n\n\"")
    # file.close()

# def listener():
#     # print("in listener")
#     rospy.Subscriber("base_scan", LaserScan, callback)
#     # print("out listener")
#     # return LaserScan().ranges
#     rospy.spin()

# x_vel = 0
# angular_vel = 0
    

def mapmover():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
    rospy.init_node("Map_Mover", anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # global angular_vel
    # global x_vel
    while not rospy.is_shutdown():
        # laser_range = []
        base_data = Twist()
        # base_data.linear.x = 0.1
        # listener()

        filereader = open("direction.txt", "r")
        direction = filereader.read(-1)
        filereader.close()

        if direction == "l":
            base_data.angular.z = -(3.14/4)
        elif direction == "x":
            base_data.angular.z = -(3.14/8)
        elif direction == "c":
            base_data.linear.x = 0.5
        elif direction == "y":
            base_data.angular.z = (3.14/8)
        elif direction == "r":
            base_data.angular.z = (3.14/4)


        
        pub.publish(base_data)
        # rate.sleep()

if __name__ == "__main__":
    try:
        mapmover()
    except rospy.ROSInterruptException:
        pass
