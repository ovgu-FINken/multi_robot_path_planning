#!/usr/bin/env python
import rospy
import traceback
#import pdb
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi, cos, sin, ceil, degrees

class TangentBug:

    state_dict = {
        0: 'motion to goal',
        1: 'follow the wall',
        2: 'goal reached'
    }

    # step_size = velocity * 1/rate
    # max_step_size = max_vel * 1/rate = 0.22 * 0.05 = 0.011m
    # vision_radius has to be bigger than step_size + max_step_size + 2*robot_radius
    # vision radius shouldn't be smaller than 0.2 with the above parameters to avoid collisions between two robots
    def __init__(self, rate = 20, vision_radius = 1, max_vel = 0.22):

        #Initialize node
        rospy.init_node('tangentBug', anonymous=True)

        #Initialize publisher
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)

        #Initialize subscriber
        self.pos_subscriber = rospy.Subscriber('odom',
                                                Odometry, self.callback_odom)
        self.finished_subscriber = rospy.Subscriber('benchmark/finished',
                                                Bool, self.callback_finished)
        self.scan_subscriber = rospy.Subscriber('scan', LaserScan,
                                                self.callback_laser)
        self.goal_subscriber = rospy.Subscriber('benchmark/waypoint', Point,
                                                self.callback_target)

        #Initialize variables
        self.robot_name = rospy.get_param('~robot_name')
        self.vision_radius = vision_radius
        self.max_vel = max_vel
        self.rate = rospy.Rate(rate)
        self.orientation = 0.0
        self.state = -1
        self.position = Point()
        self.dfollow = 0
        self.goal_pos = Point()
        self.waypoint = Point()
        self.start_pos = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.scanner = []
        self.frontScanner = []
        self.scannerRegions = []
        self.heuristicDistance = -1
        self.robot_length = 0.3
        self.robotGap = self.calcRobotGap()
        self.region = ''

###################CALLBACKS####################################################

    def callback_target(self, msg):
        self.goal_pos = msg

    def callback_finished(self, msg):
        self.finished = msg

    def callback_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x,
                                                       orientation.y,
                                                       orientation.z,
                                                       orientation.w])
        self.orientation = theta # range [-pi,pi]

    def callback_laser(self, data):
        self.scanner = data.ranges[:]
        self.frontScanner = min(min(self.scanner[360-self.robotGap/2:359]), min(self.scanner[0:self.robotGap/2]))
        self.scannerRegions = {
        'front' : min(min(data.ranges[0:35]),min(data.ranges[325:359]),self.vision_radius),
		'fleft' : min(min(data.ranges[36:107]),self.vision_radius),
		'left' : min(min(data.ranges[108:179]),self.vision_radius),
		'right' : min(min(data.ranges[180:251]),self.vision_radius),
		'fright' : min(min(data.ranges[252:324]),self.vision_radius)
    }

####################HELPER-METHODS##############################################
    def calcRobotGap(self):
        alpha = (self.robot_length * 2 )/ self.vision_radius
        return int(ceil(degrees(alpha)))

    def isEven(self, number):
        mod = number % 2
        if mod > 0:
            return False
        else:
            return True

    def change_state(self, state):
        if state is not self.state:
            print ('[%s] - State %s: %s' % (self.robot_name, state, self.state_dict[state]))
            self.state = state

    def turnToPoint(self, point, tolerance = 0.01):
        while (fabs(self.angle_error(point)) > tolerance):
            if (self.angle_error(point) < 0):
                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = -0.5
            else:
                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = 0.5
            self.velocity_publisher.publish(self.twist_msg)
            self.rate.sleep()
        self.velocity_publisher.publish(self.stop())

    # Angle error for the P-Controller
    def angle_error(self, goal_pos):
        goal_angle = atan2(goal_pos.y - self.position.y, goal_pos.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = angle - (2 * pi * angle) / (fabs(angle))
        return angle

    # The robots distance to the goal
    def euclidean_distance(self, pos, goal_pos):
        return sqrt(pow((goal_pos.x - pos.x), 2) + pow((goal_pos.y - pos.y), 2))

    def calculateNextWaypoint(self):
        points = self.getDiscontinuityPoints()
        #pdb.set_trace()
        heuristic = self.euclidean_distance(self.position, points[0]) + self.euclidean_distance(points[0], self.goal_pos)
        nextPoint = points[0]
        for i in range(1,len(points)):
            temp = self.euclidean_distance(self.position, points[i]) + self.euclidean_distance(points[i], self.goal_pos)
            if temp < heuristic:
                heuristic = temp
                nextPoint = points[i]
        if self.heuristicDistance == -1:
            self.heuristicDistance = heuristic
        else:
            if heuristic > self.heuristicDistance:
                return None
            else:
                self.heuristicDistance = heuristic

        return nextPoint

    def getDiscontinuityPoints(self):
        points = []
        scanner = []
        negOverflow = self.scanner[360-self.robotGap: 360]
        posOverflow = self.scanner[0:self.robotGap]
        scanner.extend(negOverflow)
        scanner.extend(self.scanner[:])
        scanner.extend(posOverflow)
        flag = -99
        i = self.robotGap


        if scanner[i] >= (self.vision_radius):
            flag = 0 #no Obstacle
        else:
            flag = 1 #Obstacle

        i += 1
        while i <= (360 + self.robotGap):
            if scanner[i] >= (self.vision_radius):
                tempFlag = 0
                #negflag
                if tempFlag != flag:
                    flag = tempFlag
                    if min(scanner[i:i+self.robotGap]) >= self.vision_radius:
                        alpha_deg = i - self.robotGap / 2.0 # i-self.robotGap + 0.5 * robotGap
                        if alpha_deg > 359.0:
                            alpha_deg = alpha_deg - 360.0
                        alpha_rad = self.normalize(radians(alpha_deg))
                        x = self.position.x + self.vision_radius * cos(self.orientation + alpha_rad)
                        y = self.position.y + self.vision_radius * sin(self.orientation + alpha_rad)
                        points.append(Point(x, y, 0.0))
            else:
                tempFlag = 1
                #posflag
                if tempFlag != flag:
                    flag = tempFlag
                    if min(scanner[i-self.robotGap:i]) >= self.vision_radius:
                        alpha_deg = i - self.robotGap * 1.5 #i -self.robotGap - 0.5 * self.robotGap
                        if alpha_deg < 0.0:
                            alpha_deg = alpha_deg + 360.0
                        alpha_rad = self.normalize(radians(alpha_deg))
                        x = self.position.x + self.vision_radius * cos(self.orientation + alpha_rad)
                        y = self.position.y + self.vision_radius * sin(self.orientation + alpha_rad)
                        points.append(Point(x, y, 0.0))
            i += 1

       # pdb.set_trace()
        return points

    def getDFollowed(self):
        scanner = self.scanner[:]
        d = -1
        point = Point()
        
        for i in range(360):
            if scanner[i] <= self.vision_radius:
                alpha = self.normalize(radians(i))
                point = Point(self.position.x + cos(alpha+self.orientation) * scanner[i], self.position.y + sin(alpha+self.orientation) * scanner[i], 0)
                if  d == -1:
                    d = self.euclidean_distance(point, self.goal_pos)
                else:
                    dTemp = self.euclidean_distance(point, self.goal_pos)
                    if dTemp < d:
                        d = dTemp
        return d

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.3
        return msg

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.3
        return msg

    def move_left(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.3
        return msg

    def move_right(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = -0.3
        return msg

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.1
        return msg

    # Stops the robot
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        return msg

    def moveToPoint(self, point):
        msg = Twist()
        msg.angular.z = 2 * self.angle_error(point)
        msg.linear.x = self.max_vel
        return msg


#####################STATE-METHODS##############################################

    # State 0:
    def motionToGoal(self):
        self.turnToPoint(self.goal_pos)
        nextPoint = Point()
        const = 0.0
        self.waypoint = self.goal_pos
        again = True

        while (again):
            if self.euclidean_distance(self.position,  self.goal_pos) < self.vision_radius:
                print('A')
                self.waypoint = self.goal_pos
                self.turnToPoint(self.goal_pos)
            if self.waypoint == self.goal_pos:
                print('B')
                if self.frontScanner > (self.vision_radius):
                    print('B1')
                    if self.euclidean_distance(self.position, self.goal_pos) > self.vision_radius:
                        print('B11')
                        const = sqrt(pow(self.vision_radius, 2) / (pow(self.goal_pos.x - self.position.x, 2) + pow(self.goal_pos.y - self.position.y, 2)))
                        self.waypoint.x =  self.position.x + const * (self.goal_pos.x - self.position.x)
                        self.waypoint.y =  self.position.y + const * (self.goal_pos.y - self.position.y)
                        self.waypoint.z = 0
                        while(self.euclidean_distance(self.position, self.waypoint) > 0.5):
                            self.velocity_publisher.publish(self.moveToPoint(self.waypoint))
                            self.rate.sleep()
                        self.velocity_publisher.publish(self.stop())
                        self.rate.sleep()
                        self.waypoint = self.goal_pos
                    else: 
                        print('B12')
                        while(self.euclidean_distance(self.position, self.waypoint) > 0.1):
                            self.velocity_publisher.publish(self.moveToPoint(self.waypoint))
                            self.rate.sleep()
                        self.velocity_publisher.publish(self.stop())
                        self.rate.sleep()
                        again = False
                elif self.frontScanner > self.euclidean_distance(self.position,  self.goal_pos):
                    print('B2')
                    while(self.euclidean_distance(self.position, self.waypoint) > 0.1):
                        self.velocity_publisher.publish(self.moveToPoint(self.waypoint))
                        self.rate.sleep()
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
                    again=False
                else:
                    print('B3')
                    nextPoint = self.calculateNextWaypoint()
                    if nextPoint is not None:
                        print('B31')
                        self.waypoint = nextPoint
                        self.turnToPoint(self.waypoint)
                        while(self.euclidean_distance(self.position, self.waypoint) > 0.5):
                            self.velocity_publisher.publish(self.moveToPoint(self.waypoint))
                            self.rate.sleep()
                        self.velocity_publisher.publish(self.stop())
                        self.rate.sleep()
                       # if self.angle_error(self.position) <= 0.1:
                          #  print('B311')
                        #    self.waypoint = self.goal_pos
                    else:
                        print('B32')
                        again = False
                        self.change_state(1)
            else:
                print('C')
                nextPoint = self.calculateNextWaypoint()
                if nextPoint is not None:
                    print('C1')
                    self.waypoint = nextPoint
                    self.turnToPoint(self.waypoint)
                    while(self.euclidean_distance(self.position, self.waypoint) > 0.5):
                        self.velocity_publisher.publish(self.moveToPoint(self.waypoint))
                        self.rate.sleep()
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
                  #  if self.angle_error(self.position) <= 0.1:
                  #      print('C11')
                  #      self.waypoint = self.goal_pos
                else:
                    print('C2')
                    again = False
                    self.change_state(1)

    # # State 2:
    def followWall(self, d = 0.3, tolerance = 0.01):
        #pdb.set_trace()
        if self.orientation > 0:
            self.region = 'right'
        else:
            self.region = 'left'
        msg = Twist()
        again = True
        self.dfollow = self.getDFollowed()

        while (again):
            if self.scannerRegions['front'] > d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] > d and self.scannerRegions[self.region] > d:
                #state_description = 'case 1 - nothing'
                if self.region == 'right':
                    msg = self.move_left()
                else:
                    msg = self.move_right()
            elif self.scannerRegions['front'] < d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] > d:
                #state_description = 'case 2 - front'
                if self.region == 'right':
                    msg = self.turn_left()
                else:
                    msg = self.turn_right()
            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] < d and self.scannerRegions[self.region] > d:
            #state_description = 'case 3 - fright'
                if self.region == 'right':
                    msg = self.move_forward()
                else:
                    msg = self.move_left()
            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] < d and self.scannerRegions['fright'] > d and self.scannerRegions[self.region] > d:
            #state_description = 'case 4 - fleft'
                if self.region == 'right':
                    msg = self.move_right()
                else:
                    msg = self.move_forward()
            elif self.scannerRegions['front'] < d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] < d:
            #state_description = 'case 5 - front and fright'
                if self.region == 'right':
                    msg = self.turn_left()
                else:
                    msg = self.move_left()
            elif self.scannerRegions['front'] < d and self.scannerRegions['fleft'] < d and self.scannerRegions['fright'] > d:
            #state_description = 'case 6 - front and fleft'
                if self.region == 'right':
                    msg = self.move_right()
                else:
                    msg = self.turn_right()
            elif self.scannerRegions['front'] < d and self.scannerRegions['fleft'] < d and self.scannerRegions['fright'] < d:
            #state_description = 'case 7 - front and fleft and fright'
                if self.region == 'right':
                    msg = self.turn_left()
                else:
                    msg = self.turn_right()
            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] < d and self.scannerRegions['fright'] < d:
            #state_description = 'case 8 - fleft and fright'
                msg = self.move_forward()


            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] > d and self.scannerRegions[self.region] < d:
                #state_description = 'case 9 - right or left'
                if self.region == 'right':
                    msg = self.turn_right()
                else:
                    msg = self.turn_left()
            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] > d and self.scannerRegions['fright'] < d and self.scannerRegions[self.region] < d:
            #state_description = 'case 10 - fright and right or left'
                if self.region == 'right':
                    msg = self.move_right()
                else:
                    msg = self.move_forward()
            elif self.scannerRegions['front'] > d and self.scannerRegions['fleft'] < d and self.scannerRegions['fright'] > d and self.scannerRegions[self.region] < d:
            #state_description = 'case 10 - fleft and right or left'
                if self.region == 'right':
                    msg = self.move_forward()
                else:
                    msg = self.move_left()
            else:
                #state_description = 'unknown case'
                rospy.loginfo(self.scannerRegions)

            self.velocity_publisher.publish(msg)
            self.rate.sleep()
            dreach = self.euclidean_distance(self.position, self.goal_pos)

            if (dreach) < self.dfollow:
                msg = self.stop()
                self.velocity_publisher.publish(msg)
                self.rate.sleep()
                self.change_state(0)
                self.waypoint = self.goal_pos
                again = False

    #State 4: Goal reached
    def goalReached(self):
        self.heuristicDistance = -1
        self.velocity_publisher.publish(self.stop())
        self.rate.sleep()
    
    #State3: Collision
    def robotCollision(self):
      scanner= self.scanner[0:91]
      beta_degree = 0
      point = Point()
      for i in range(len(scanner)):
            if scanner[i] >= self.vision_radius:
                alpha = int(ceil(degrees(((0.01+0.5*self.robot_legth) * 2 )/ self.vision_radius)))
                beta_degree = i + alpha
                beta_rad = self.normalize(radians(beta_degree))
                x = self.position.x + self.vision_radius * cos(self.orientation + beta_rad)
                y = self.position.y + self.vision_radius * sin(self.orientation + beta_rad)
                point = Point(x, y, 0.0)
                break
        if beta_degree != 0 and (self.euclidean_distance(self.position, point) + 0.3) < scanner[beta_degree]:
            while self.euclidean_distance(self.position, point) > 0.1:
                self.velocity_publisher.publish(self.moveToPoint(point))
                self.rate.sleep()
            self.velocity_publisher.publish(self.stop())
            self.rate.sleep()
            self.change_state(0)
        else:
            self.rate.sleep()
            self.change_state(0)
 

###########################THE ALGORITHM########################################
    def tangentBug(self, tolerance = 0.1):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            #Robots has found all goals
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Done! ' % (self.robot_name))
            #Robot is waiting for a new goal
            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Waiting for a new goal' % (self.robot_name))
            #Robot has received a new goal
            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                print('[%s] - Goal received' % (self.robot_name))
                new_goal_pos = self.goal_pos
                self.waypoint = self.goal_pos
                #self.start_pos =  self.position
                print('[%s] - Turning to goal' % (self.robot_name))
                self.turnToPoint(self.goal_pos)
                self.change_state(0)

                #Robot is driving towards the goal
                while self.euclidean_distance(self.position, self.goal_pos) > tolerance:
                    if self.state == 0:
                        self.motionToGoal()
                    elif self.state == 1:
                        self.followWall()

                #Robot has found the current goal
                self.change_state(2)
                self.goalReached()

            self.rate.sleep()

##############################MAIN##############################################

if __name__ == "__main__" :

    try:
        x = TangentBug()
        x.tangentBug()
    except rospy.ROSInterruptException:
        traceback.print_exc()
