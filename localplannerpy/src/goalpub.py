#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('goal_fake')
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
info = PoseStamped()
info.pose.position.x = 1.97424972057
info.pose.position.y = -0.861016988754
info.pose.position.z = 0.0

info.pose.orientation.x = 0.0
info.pose.orientation.y = 0.0
info.pose.orientation.z = -0.193779598787
info.pose.orientation.w = 0.981045089226

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    pub.publish(info)
    rate.sleep()