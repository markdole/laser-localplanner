#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
rospy.init_node('odom_fake')
pub = rospy.Publisher('/odom', Odometry, queue_size=1)

info = Odometry()

info.pose.pose.position.x = -1.99999336044
info.pose.pose.position.y = -0.499968668593
info.pose.pose.position.z = -0.00100739907919
info.pose.covariance =  [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

info.pose.pose.orientation.x = -1.81703459728e-06
info.pose.pose.orientation.y = 0.00158965006883
info.pose.pose.orientation.z = 9.78100656928e-05
info.pose.pose.orientation.w = 0.99999873172

info.twist.twist.linear.x = 0.0
info.twist.twist.linear.y = 0.0
info.twist.twist.linear.z = 0.0

info.twist.twist.angular.x= 0.0
info.twist.twist.angular.y= 0.0
info.twist.twist.angular.z= 0.0

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    pub.publish(info)
    rate.sleep()