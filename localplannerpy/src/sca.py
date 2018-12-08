#!/usr/bin/env python
import rospy, numpy
from numpy import inf
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf import transformations
class LocalPlanner():

    def __init__(self):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size = 1)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cbGoal, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.cbLaser, queue_size=1)
        self.goal = PoseStamped()
        self.odom = Odometry()
        self.front_obstacle = ([], [])
        self.is_free_to_go = True
        goal_point = numpy.array([self.goal.pose.position.x, self.goal.pose.position.y])
        self.list_H = [goal_point]
        self.list_T = [goal_point]

    def cbOdom(self, msg):
        self.odom = msg

    def cbGoal(self, msg):
        self.goal = msg

    def check_path_across(self):
        theta = self.o_g_theta - self.odom_yaw
#        right_part = []
#        left_part = []
#        for i in self.front_obstacle[1]:
#            angle = self.bean_to_angle(i[1])
#            if angle < theta:
#                right_part.append(self.front_obstacle[1][self.front_obstacle[1].index(i):])
#                break
#            else:
#                left_part.append(i)
        right_part = []
        left_part = []
        theta_bean = self.angle_to_bean(theta)
        if self.front_obstacle[1][0][1] < 0 and theta_bean < 0:
            theta_bean -= 360
        while len(self.front_obstacle[1]) / 2 > 0:
            if self.front_obstacle[1][len(self.front_obstacle[1]) / 2][1] > theta_bean:
                add_list = self.front_obstacle[1][len(self.front_obstacle[1]) / 2:]
                add_list.reverse()
                right_part.extend(add_list)
                del self.front_obstacle[1][len(self.front_obstacle[1]) / 2: ]
            elif self.front_obstacle[1][len(self.front_obstacle[1]) / 2][1] < theta_bean:
                add_list = self.front_obstacle[1][:len(self.front_obstacle[1]) / 2 + 1]
                left_part.extend(add_list)
                del self.front_obstacle[1][len(self.front_obstacle[1]) / 2:]
            else:
                right_part.extend(self.front_obstacle[1][len(self.front_obstacle[1])/2:])
                right_part.reverse()
                left_part.extend(self.front_obstacle[1][:len(self.front_obstacle[1]) /2 +1])
                self.front_obstacle = ([],[])
        #dichotomy algrithmn
        v_g = self.pol2cart([self.o_g_distance, self.o_g_theta])
        #------------#
        v_pm = self.pol2cart(left_part[-1])
        v_pmg_max = v_g - v_pm
        theta_p_max = numpy.arctan2(v_pmg_max[0], v_pmg_max[1])
        for i in left_part[:-1]:
            v_pi = self.pol2cart(i)
            theta_pi = numpy.arctan2(v_pi[0], v_pi[1])
            if theta_pi > theta_p_max:
                self.list_H.append(v_pi)  # the ansewer
        self.list_H.append(v_pm)
        #------------#
        v_qm = self.pol2cart(right_part[0])
        v_qg_max = v_g - v_qm
        theta_q_max = numpy.arctan2(v_qg_max[0], v_qg_max[1])
        for i in right_part[1:]:
            v_qi = self.pol2cart(i)
            theta_qi = numpy.arctan2(v_qi[0], v_qi[1])
            if theta_qi > theta_q_max:
                self.list_T.append(v_qi)
        self.list_T.append(v_qm)
        # send to the Path

    def pol2cart(self, i):
        dis = i[0]
        ang = self.odom_yaw + self.bean_to_angle(i[1])
        x = dis*numpy.cos(ang)
        y = dis*numpy.sin(ang)
        return numpy.array([x, y])

    def angle_to_bean(self, ang):
        if ang < 0.0:
            bean = - round(ang*180.0/numpy.pi)
        else:
            bean = 360 - round(ang*180.0/numpy.pi)
        return bean

    def bean_to_angle(self,bean):
        if bean > 180.0:
            angle = (360.0 - bean)/180.0*numpy.pi
            return angle
        elif bean < 180.0:
            angle = - bean/180.0 * numpy.pi
            return angle

    def obs_detect(self, items, link_angle, odom_angle):
        left_angle = self.bean_to_angle(items[1][0][1])
        right_angle = self.bean_to_angle(items[1][-1][1])
        if (odom_angle + left_angle) > link_angle and (odom_angle + right_angle) < link_angle:
            return True
        else:
            return False

    def point_resample(self, item):  # change a more sufficient way to resample depending on the subjective distance
        item_position = item[1]
        mean_dis = numpy.mean( item_position,0)[0]
        n_i = len(item_position)
        ang_step = round(180/(numpy.pi*mean_dis))
        my_slice = slice(0, n_i, int(ang_step))
        if n_i > 4:
            i_resample = item_position[my_slice]
        else:
            i_resample = [item_position[0], item_position[-1]]
        return i_resample

    def point_rename(self, Point_set, obs_id):
        i = 0
        n = len(obs_id)
        while i < n:
            if i not in obs_id:
                Point_set[i] = Point_set[max(obs_id)]
                Point_set.pop(obs_id.pop())
            i += 1
        return Point_set

    def point_distinguish(self, points):
        Points_free = {}
        free_id = []
        obs_id = []
        link_a, odom_a, = self.link_angle()
        for i in points.items():
            if i[1][0][0] == inf:
                Points_free[i[0]] = points.pop(i[0])
                free_id.append(i[0])
            else:
                obs_id.append(i[0])
                if self.obs_detect(i, link_a, odom_a):
                    self.front_obstacle = (i[0], self.point_resample(i))
        if len(self.front_obstacle[1] ) == 2:  # fix me :any danger ?
            self.is_free_to_go = True
        else:
            self.is_free_to_go = False
        return points
        #else:# else if no obstale ?

    def detect_block(self):
        range_tolerence = 0.5
        scan_arr = numpy.array(self.point_set)
        diff_1 = numpy.abs(numpy.diff(scan_arr, 1))
        diff_list = diff_1.tolist()
        del diff_1, scan_arr
        # jump_index = []
        point_set = {}
        conter = 0
        points_range = []
        for i in diff_list:
            if i > range_tolerence:
                # jump_index.append(diff_list.index(i))
                conter += 1
                points_range = []
            else:
                points_range.append([self.point_set[diff_list.index(i)], diff_list.index(i)])
                point_set[conter] = points_range

        if abs(self.point_set[-1] - self.point_set[0]) < range_tolerence:
            cut_point = point_set.pop(conter)
            for i in cut_point:
                i[1] -= 360
            cut_point.extend(point_set[0])
            point_set[0] = cut_point

        self.point_set = self.point_distinguish(point_set)  # check the obstacle on the line to goal

        if self.is_free_to_go:
            return
        #  is free to go
        else:
            self.check_path_across( )
            return

    def link_angle(self):
        self.goal_pos = self.goal.pose.position
        self.odom_pos = self.odom.pose.pose.position
        self.o_g_theta = numpy.arctan2(self.goal_pos.y - self.odom_pos.y, self.goal_pos.x - self.odom_pos.x)
        self.odom_yaw = self.orientation_to_theta(self.odom.pose.pose.orientation)
        self.o_g_distance = numpy.linalg.norm(numpy.array([self.goal_pos.x, self.goal_pos.y]) - numpy.array([self.odom_pos.x , self.odom_pos.y]))
        return [self.o_g_theta, self.odom_yaw]

    def orientation_to_theta(self, orientation):
        ang = transformations.euler_from_quaternion((orientation.x,
                                                    orientation.y,
                                                    orientation.z,
                                                    orientation.w))
        return ang[2]

    def cbLaser(self, msg):
        self.point_set = msg.ranges
        self.detect_block()
        # print POint.items()[0][1]

rospy.init_node('Im_genius')
LocalPlanner()
rospy.spin()