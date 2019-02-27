#!/usr/bin/env python
import rospy
from PIL import Image, ImageDraw
import math
from sensor_msgs.msg import Imu,JointState
from geometry_msgs.msg import Twist,Pose2D ,PoseStamped
from nav_msgs.msg import Path,Odometry
import time
class Pn():
    def __init__(self, x , y):
        self.x = x
        self.y = y
    def norm(self):
        self.pix = self.x * 400
        self.piy = self.y * 400

def B(coorArr, i, j, t):
    if j == 0:
        return coorArr[i]
    return B(coorArr, i, j - 1, t) * (1 - t) + B(coorArr, i + 1, j - 1, t) * t

#############


def point(pose1 = Pose2D(), pose2 = Pose2D(),imu = Imu()):
    global n
    n = 5 # number of control points
    w = imu.angular_velocity.z
    v_a = imu.linear_acceleration
    P = range(5)
    x2 = (pose2.x - pose1.x) /2 # mid of the distance
    d1 = 0.1 ## define by the initial velocity
    d4 = 0.1 # define by the demanding velocity of the subgoal and the w_
    k = 3.1 ## the first turning arc
    P[4] = Pn(pose2.x - pose1.x , pose2.y - pose1.y)
    psi = pose2.theta - pose1.theta ## the goal yaw rate
    P[0] = Pn(0.0, 0.0)
    P[1] = Pn(d1, 0.0 )
    P[2] = Pn(x2 , 4.0 * k * d1 *d1 /3.0)
    P[3] = Pn( P[4].x - d4 * math.cos( psi ), P[4].y - d4* math.sin( psi ))
    coorArrX = []
    coorArrY = []
    ctrlpx = []
    ctrlpy = []
    for k in range(n):
        P[k].norm()
        x = P[k].pix +20
        y = P[k].piy +20
        coorArrX.append(x)
        coorArrY.append(y)
        ctrlpx.append(P[k].x)
        ctrlpy.append(P[k].y)
    return coorArrX,coorArrY,ctrlpx,ctrlpy

class Robot_state():
    def __init__(self):
        self.v = 0
        self.w = 0
        self.theta = 0

def xy_U(x= [], y = []):
    n = len(x)
    dx = []
    dy = []
    ddx = []
    ddy = []
    state = Robot_state()
    state_line = []
    v_line = []
    w_line = []
    h = 0.1 # todo: the meaning of step h ralation to time  h = 0.1s = '10Hz publish rate '
    for i in range(n-2):
        dx.append ((x[i+2] - x[i]) / (2 * h))
        dy.append ((y[i+2] - y[i]) / (2 * h))
        ddx.append((x[i + 2] - 2 * x[i+1] + x[i]) / (h * h))
        ddy.append((y[i + 2] - 2 * y[i+1] + y[i]) / (h * h))

        state.v = math.sqrt(math.pow(dx[i], 2) + math.pow(dy[i], 2))
        state.w = (ddy[i] * dx[i] - ddx[i]*dy[i]) / (math.pow(dx[i], 2) + math.pow(dy[i], 2))
        state.theta = math.atan2(dy[i], dx[i])
        state_line.append([state.v,state.w, state.theta])
        v_line.append(state.v)
        w_line.append(state.w)
        i += 1
    return state_line,v_line

####################
# plot the curve


def track_out( numSteps,coorArrX,coorArrY,ctrlpx,ctrlpy,image):
    trak = []
    dotx = []
    doty = []
    for k in range(numSteps):
        t = float(k) / (numSteps - 1)
        x = int(B(coorArrX, 0, n - 1, t))
        y = int(B(coorArrY, 0, n - 1, t))
        x_f = B(ctrlpx, 0, n - 1, t)
        y_f = B(ctrlpy, 0, n - 1 ,t)
        try:
            #image.putpixel((x, y), (0, 255, 0))
            trak.append((x, y))
            dotx.append(x_f)
            doty.append(y_f)
        except:
            pass
    return dotx,doty,trak

def main():
    imgx = 600
    imgy = 600
    image = Image.new("RGB", (imgx, imgy))
    draw = ImageDraw.Draw(image)
    v_lim = 0.2 # the max vel of robot
    pose1 = Pose2D()
    pose1.x = 0.0
    pose1.y = 0.0
    pose1.theta = 0.5
    pose2 = Pose2D()
    pose2.x = 1.0
    pose2.y = 0.5
    pose2.theta = -0.5
    imu = Imu()
    coorArrX, coorArrY, ctrlpx, ctrlpy = point(pose1,pose2,imu)
    numSteps = int((abs(pose2.x- pose1.x) + abs(pose2.y- pose2.y)) * 50)
    dotx,doty,trak= track_out(numSteps,coorArrX, coorArrY, ctrlpx, ctrlpy,image)
    state_, v_line = xy_U(dotx, doty)
    for i in range(1,30):
        if max(v_line) > v_lim:
            numSteps += 3
            dotx, doty,trak = track_out(numSteps,coorArrX, coorArrY, ctrlpx, ctrlpy,image)
            state_, v_line = xy_U(dotx, doty)
        elif max(v_line) < (v_lim-0.03):
            numSteps -= 3
            dotx, doty ,trak= track_out(numSteps,coorArrX, coorArrY, ctrlpx, ctrlpy,image)
            state_, v_line = xy_U(dotx, doty)
        else:
            break
    plot_cotrl(coorArrX,coorArrY,draw)
    v_acc = []
    for i in range(numSteps-3):
        acc = (v_line[i + 1 ]- v_line[i])/0.1
        v_acc.append(acc)
    v_acc.insert(0,v_acc[0])
    v_acc.insert(-1,v_acc[-1])
    for i in trak:
        image.putpixel((i[0], i[1]), (0, 255, 0))
    for i in state_:
        step  = state_.index(i)
        image.putpixel( (step*5 + 10 , int(i[1]*80 + 250) ) , (255,100,0)) #orange line for angular_v
        image.putpixel( (step*5 + 10 , int(i[0]*100 + 250) ) , (0,100,255)) #blue line for linear_v
        image.putpixel( (step*5 + 10 , int(v_acc[step]*100 + 250)) , (255,0,100))# pink line for linear_acc
        image.putpixel( (step*5 +10 ,250),(255, 255, 255))
    print 'max', max(v_line), 'first', v_line[0] ,numSteps,'steps ','first_w', state_[0][1] ,time.time()
    #image.show() # take 18ms
    #image_v.show()
#    image.save("BezierCurve.png", "PNG") ## using 26 ms
    path = Path()
    path.header.frame_id = "base_link"
    for i in range(numSteps):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        # todo: stamp info
        pose.pose.position.x = dotx[i]
        pose.pose.position.y = doty[i]
        path.poses.append(pose)
    return path,state_

# plot the control points
def plot_cotrl(coorArrX,coorArrY,draw):
    cr = 3 # circle radius
    n = len(coorArrX)
    for k in range(n):
        x = coorArrX[k]
        y = coorArrY[k]
        try:
            draw.ellipse((x - cr, y - cr, x + cr, y + cr), (255, 0, 0))
        except:
            pass

if __name__ == '__main__':
    rospy.init_node('path')
    pathpub = rospy.Publisher('path',Path,queue_size=1)
    cmd_v = rospy.Publisher("cmd_vel", Twist,queue_size=1)
    path,state_ = main()
    state_.append([0,0,0])
    rate = rospy.Rate(10)
    pathpub.publish(path)
    cmd_ = []
    for i in state_:
        cmd = Twist()
        cmd.linear.x = i[0]
        cmd.angular.z = i[1]
        cmd_.append(cmd)
    i = 0
    pathpub.publish(path)
    while not rospy.is_shutdown():
        cmd_v.publish(cmd_[i])
        i+=1
        rate.sleep()
## turn into Class style fuction