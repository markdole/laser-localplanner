#!/usr/bin/env python
import rospy, numpy
from numpy import inf
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist,Pose2D
from tf import transformations
class LocalPlanner():

    def __init__(self):
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size = 1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.cbLaser, queue_size=1)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cbGoal, queue_size=1)
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        self.pub_path = rospy.Publisher('/path',Path, queue_size=1)
        self.goal = PoseStamped()
        self.odom = Odometry()
        self.goal_pose_2d = Pose2D()
        self.twist = Twist()
        self.front_obstacle = ([], [])
        self.is_free_to_go = True
        self.finished = False
        self.interupt = False
        # done: clarify the class of path points

    def cbOdom(self, msg):
        self.odom = msg
        self.link_angle()


    def cbLaser(self, msg):
        self.point_set = msg.ranges

    def cbGoal(self, msg):
        self.interupt = True
        self.goal = msg
        self.link_angle()
        self.goal_pose_2d.x = self.goal.pose.position.x
        self.goal_pose_2d.y = self.goal.pose.position.y
        self.goal_pose_2d.theta = self.goal_yaw
        self.path = [self.goal_pose_2d]
        self.list_H = [[self.goal_pose_2d,[0.0, 0.0]]]
        self.list_T = [[self.goal_pose_2d,[0.0, 0.0]]]
        self.detect_block()
        self.letsmove()## fixme: move rate and turning behavior

    def move_core(self, goal_pi= Pose2D()):
        Kp_v = 0.1
        Ki_v = 0.1
        last_linear_x_err = 0.0
        Kp_a = 1
        Ki_a = 1
        last_angle_err = 0.0
        while not self.finished:
            #---------------# Done: deal with the callback fcn
            angle_error= goal_pi.theta - self.odom_pos_2d.theta
            angle_increase = Kp_a * (angle_error - last_angle_err) + Ki_a * angle_error
            angular_z_demand = cmp(angle_increase, 0 )* (min( abs(angle_increase)* 0.86 , 1.82))
            self.twist.angular.z = angular_z_demand* 2
            last_angle_err = angle_error

            # TODO: better angle aligorithm to deal with
            distance = abs(goal_pi.x - self.odom_pos_2d.x)+ abs(goal_pi.y - self.odom_pos_2d.y)
            linear_x_demand = min(distance*0.1, 0.26) * numpy.cos(angle_error)
            linear_x_err = linear_x_demand - self.odom.twist.twist.linear.x
            linear_x_increase = Kp_v * (linear_x_err- last_linear_x_err) + Ki_v * linear_x_err
            #self.twist.linear.x = self.odom.twist.twist.linear.x + linear_x_increase
            last_linear_x_err = linear_x_err
            #----------------#
            if distance < 0.1:
                self.finished = True
            else:
                self.finished = False
            if self.interupt:
                break
            else:
                self.pub_twist.publish(self.twist)

        #TODO: move core function with PID


    def letsmove(self):
        self.interupt = False
        while not self.is_free_to_go:
            self.move_core(self.path[-1])
            self.path.pop()
            self.detect_block()
        self.move_core(self.path[0])
        self.path.pop()

    def check_path_across(self):
        theta = self.o_g_theta - self.odom_pos_2d.theta
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
        #dichotomy algrithm

        #____________ detect trp Point _____________________________#
        v_g= self.pol2cart([self.o_g_distance, self.o_g_theta])[0]
        #-----------#
        v_pm, ang_p= self.pol2cart(left_part[0])
        v_pmg_max = v_g - v_pm
        theta_p_max = numpy.arctan2(v_pmg_max[0], v_pmg_max[1])
        # 4 dimenstion theta ?
        for i in left_part[1:]:
            v_i, ang_i= self.pol2cart(i)
            v_pi = v_g - v_i
            v_ppm = v_pm - v_i
            theta_pi = numpy.arctan2(v_pi[0], v_pi[1])
            if theta_pi > theta_p_max:
                pi_pose = self.arr_to_pose2D(v_i, ang_i)
                self.list_H.append([pi_pose, [numpy.linalg.norm(v_pi), numpy.linalg.norm(v_ppm)]])
                break # the ansewe
        p_p_pose = self.arr_to_pose2D(v_pm, ang_p)
        self.list_H.append([p_p_pose,[numpy.linalg.norm(v_pmg_max), left_part[0][0]]])
        #------------#
        v_qm, ang_q = self.pol2cart(right_part[0])
        v_qg_max = v_g - v_qm
        theta_q_max = numpy.arctan2(v_qg_max[0], v_qg_max[1])
        for i in right_part[1:]:
            v_j, ang_j= self.pol2cart(i)
            v_qi = v_g - v_j
            v_qmi = v_qm - v_j
            theta_qi = numpy.arctan2(v_qi[0], v_qi[1])
            if theta_qi < theta_q_max:
                pi_pose = self.arr_to_pose2D(v_j, ang_j)
                self.list_T.append([pi_pose,[numpy.linalg.norm(v_qi), numpy.linalg.norm(v_qmi)]])
                break
        p_q_pose = self.arr_to_pose2D(v_qm, ang_q)
        self.list_T.append([p_q_pose,[numpy.linalg.norm(v_qg_max), right_part[0][0]]])
        #______________________________________________________#
        cost_right = self.sum_dis(self.list_T)
        cost_left = self.sum_dis(self.list_H)
        direction = cmp(cost_left,cost_right)
        # done: set the list to poseStamped class
        turnpoint = []
        if direction == -1:# left
            for i in self.list_H:
                turnpoint.append(i[0])
        elif direction == 1:# right
            for i in self.list_T:
                turnpoint.append(i[0])
        self.path.extend(turnpoint)# done: send out the Path
        return

    def arr_to_pose2D(self, arr, ang, pose = Pose2D()):
        pose.x = arr[0] + self.odom_pos.x
        pose.y = arr[1] + self.odom_pos.y
        pose.theta = ang
        return pose

    def sum_dis(self, list_O):
        if len(list_O) > 2:
            i_sum = sum(list_O[1][1]) + list_O[-1][1][1]
        else:
            i_sum = sum(list_O[1][1])
        return i_sum

    def pol2cart(self, i):
        dis = i[0]
        ang = self.odom_pos_2d.theta + self.bean_to_angle(i[1])
        x = dis*numpy.cos(ang)
        y = dis*numpy.sin(ang)
        return numpy.array([x, y]) , ang

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
        #TODO: dis  goal between obs an odom

    def point_resample(self, item):  # change a more sufficient way to resample depending on the subjective distance
        item_position = item[1]
        mean_dis = numpy.mean( item_position,0)[0]
        n_i = len(item_position)
        ang_step = max(1,round(180/(numpy.pi*mean_dis)))
        my_slice =  slice(0, n_i, int(ang_step))
        if n_i > 4:
            i_resample = item_position[my_slice]
        else:
            i_resample = [item_position[0], item_position[-1]]
        return i_resample

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
        if len(self.front_obstacle[1] ) < 2:  # fixme :any danger ?
            self.is_free_to_go = True
        else:
            self.is_free_to_go = False
        return points
        #else:#  else if no obstale ?

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
                points_range.append([self.point_set[diff_list.index(i)], diff_list.index(i)])
                point_set[conter] = points_range
                points_range = []
                conter += 1

            else:
                points_range.append([self.point_set[diff_list.index(i)], diff_list.index(i)])
                point_set[conter] = points_range

        if abs(self.point_set[-1] - self.point_set[0]) < range_tolerence:
            try:cut_point = point_set.pop(conter)
            except KeyError:
                conter -= 1
                cut_point = point_set.pop(conter)
            for i in cut_point:
                i[1] -= 360
            cut_point.extend(point_set[0])
            point_set[0] = cut_point

        self.point_set = self.point_distinguish(point_set)  # check the obstacle on the line to goal

        if self.is_free_to_go:
            self.move_core(self.path[-1])
            return
        else:
            self.check_path_across( )
            return

    def link_angle(self):
        self.goal_pos = self.goal.pose.position
        self.odom_pos = self.odom.pose.pose.position
        self.o_g_theta = numpy.arctan2(self.goal_pos.y - self.odom_pos.y, self.goal_pos.x - self.odom_pos.x)

        self.odom_pos_2d = Pose2D()
        self.odom_pos_2d.x = self.odom_pos.x
        self.odom_pos_2d.y = self.odom_pos.y
        self.odom_pos_2d.theta = self.orientation_to_theta(self.odom.pose.pose.orientation)

        self.goal_yaw = self.orientation_to_theta(self.goal.pose.orientation)
        self.o_g_distance = numpy.linalg.norm(numpy.array([self.goal_pos.x, self.goal_pos.y]) - numpy.array([self.odom_pos.x , self.odom_pos.y]))

        return [self.o_g_theta, self.odom_pos_2d.theta]

    def orientation_to_theta(self, orientation):
        ang = transformations.euler_from_quaternion((orientation.x,
                                                     orientation.y,
                                                     orientation.z,
                                                     orientation.w))
        return ang[2]


def main():
    rospy.init_node('mission_control')
    try:
        control = LocalPlanner()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()