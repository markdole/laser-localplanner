#!/usr/bin/env python
import rospy
import std_msgs
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionResult
import tf
import math

class Local_planner():
    def __init__(self):
        self.laserobj = rospy.Subscriber('/scan', LaserScan,self.cbLaser,queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size=1)
        self.sub_arrival_status = rospy.Subscriber("/move_base/result", MoveBaseActionResult,
                                               self.cbGetNavigationResult, queue_size=1)
    def cbLaser(self,msg_laser):
        self.point_sets = msg_laser.

    def IO_generat(self, goal, position, point_sets):

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def cbGetNavigationResult(self, msg_nav_result):
        if msg_nav_result.status.status == 3:
            rospy.loginfo("Reached")
            self.is_navigation_finished = True

    def cbOdom(self, odom_msg):
        quaternion = (
            odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)
        self.odom_msg = odom_msg
        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

    def fnPubGoalPose(self):
        goalPoseStamped = PoseStamped()

        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = rospy.Time.now()

        goalPoseStamped.pose.position.x = 0.15
        goalPoseStamped.pose.position.y = -1.76
        goalPoseStamped.pose.position.z = 0.0

        goalPoseStamped.pose.orientation.x = 0.0
        goalPoseStamped.pose.orientation.y = 0.0
        goalPoseStamped.pose.orientation.z = 0.0
        goalPoseStamped.pose.orientation.w = 1.0

        self.pub_goal_pose_stamped.publish(goalPoseStamped)

    def fnStraight(self, desired_dist):
        err_pos = math.sqrt(
            (self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist

        rospy.loginfo("Tunnel_Straight")
        rospy.loginfo("err_pos  desired_dist : %f  %f  %f", err_pos, desired_dist, self.lastError)

        Kp = 0.4
        Kd = 0.05

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def cbTunnelFinished(self, tunnel_finished_msg):
        self.is_tunnel_finished = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_tunnel')
    node = DetectTunnel()
    node.main()