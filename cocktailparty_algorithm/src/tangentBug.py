#!/usr/bin/env python
import rospy
import traceback
import pdb
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi, cos, sin, ceil, degrees

class TangentBug:

    state_dict = {
        0: 'Motion to goal',
        1: 'Transition', 
        2: 'Following obstacle',
        3: 'Collision avoidance', 
        4: 'Goal reached'
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
        self.end = rospy.get_param("/end_procedure")
        self.robot_name = rospy.get_param('~robot_name')
        self.vision_radius = vision_radius
        self.max_vel = max_vel
        self.rate = rospy.Rate(rate)
        self.orientation = 0.0
        self.state = -1
        self.position = Point()
        self.point = Point()
        self.points = []
        self.dfollow = 0
        self.goal_pos = Point()
        self.waypoint = Point()
        self.start_pos = Point()
        self.init_pos = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.scanner = []
        self.frontScanner = 0
        self.scannerRegions = []
        self.edgeScanner = 0
        self.heuristicDistance = -1
        self.robot_length = 0.2
        self.max_step = 0.055
        self.d = self.robot_length + 2 * self.max_step + 0.1
        self.robotGap = self.calcRobotGap()
        self.region = ''
        self.tolerance = 0.2
        self.collisionScannerSize = int(ceil(((self.robot_length +0.2) * 360) /( 2* pi * self.d)))
        if self.isEven(self.collisionScannerSize) != True :
            self.collisionScannerSize += 1

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
        self.regions ={
            'front' : min(min(data.ranges[0:30]),min(data.ranges[330:360]),self.vision_radius),
            'fleft' : min(min(data.ranges[30:90]),self.vision_radius),
            'bleft' : min(min(data.ranges[90:150]),self.vision_radius),
            'back' : min(min(data.ranges[150:210]),self.vision_radius),
            'bright' : min(min(data.ranges[210:270]),self.vision_radius), 
            'fright' : min(min(data.ranges[270:330]),self.vision_radius)
            }
        self.frontScanner =  min(min(data.ranges[0:(self.robotGap/2)]),min(data.ranges[(360 - self.robotGap / 2):360]),self.vision_radius)
        self.collisionScanner = min(min(data.ranges[0:30]),min(data.ranges[330:360]),self.vision_radius)
        #self.collisionScanner =  min(min(data.ranges[0:(self.collisionScannerSize/2)]),min(data.ranges[(360 - self.collisionScannerSize / 2):360]),self.vision_radius)
        self.edgeScanners = [min(min(self.scanner[30:60]),self.vision_radius),  min(min(self.scanner[300:330]),self.vision_radius) ]
####################HELPER-METHODS##############################################
    def calcRobotGap(self):
        alpha = int(ceil(((self.robot_length + 2*self.d) * 360) /( 2* pi * self.vision_radius)))
        if self.isEven(alpha) != True:
            alpha += 1
        return alpha

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

    def calculateNextWaypoint(self,  goal_pos):
        points = self.getDiscontinuityPoints()
        #pdb.set_trace()
        heuristic = self.euclidean_distance(self.position, points[0]) + self.euclidean_distance(points[0], goal_pos)
        nextPoint = points[0]
        for i in range(1,len(points)):
            temp = self.euclidean_distance(self.position, points[i]) + self.euclidean_distance(points[i], goal_pos)
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

    def getDFollowed(self,  goal_pos):
        scanner = self.scanner[:]
        d = -1
        point = Point()

        for i in range(360):
            if scanner[i] <= self.vision_radius:
                alpha = self.normalize(radians(i))
                point = Point(self.position.x + cos(alpha+self.orientation) * scanner[i], self.position.y + sin(alpha+self.orientation) * scanner[i], 0)
                if  d == -1:
                    d = self.euclidean_distance(point, goal_pos)
                else:
                    dTemp = self.euclidean_distance(point, goal_pos)
                    if dTemp < d:
                        d = dTemp
        return d
    def turn(self,  direction):
        msg = Twist()
        msg.angular.z = direction * 0.5
        return msg

    def move(self,  direction):
        msg = Twist()
        msg.linear.x = self.max_vel
        msg.angular.z = direction * 0.5
        return msg
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.max_vel
        return msg
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        return msg

    def moveToPoint(self, waypoint,  tolerance,  collisionAvoidance):
        msg = Twist()
        while(self.euclidean_distance(self.position, waypoint) > tolerance):
            if self.collisionScanner > self.d or collisionAvoidance == False:
                msg.angular.z = self.angle_error(waypoint)
                msg.linear.x = self.max_vel
                self.velocity_publisher.publish(msg)
                self.rate.sleep()
            else:
                msg.angular.z = 0
                msg.linear.x = 0
                self.velocity_publisher.publish(msg) 
                self.rate.sleep()
                return False
        return True
  
    def  getObstaclePosition(self):
        scanner = []
        scannerA = self.scanner[360 - self.collisionScannerSize / 2: 360]
        scannerB = self.scanner[0:self.collisionScannerSize / 2]
        scanner.extend(scannerA)
        scanner.extend(scannerB)
        
        index = scanner.index(min(scanner)) - self.collisionScannerSize / 2
            
#        alpha_rad = self.normalize(radians(index))    
#        x = self.position.x + self.vision_radius * cos(self.orientation + alpha_rad)
#        y = self.position.y + self.vision_radius * sin(self.orientation + alpha_rad)
#        point = Point(x, y, 0.0)
        return index
        
#####################STATE-METHODS##############################################

    # State 0:
    def motionToGoal(self,  goal_pos):
        self.turnToPoint(goal_pos)
        if self.frontScanner >= self.vision_radius or self.euclidean_distance(self.position,  goal_pos) < self.frontScanner:
            self.goStraight(goal_pos) #Go straight
        else:
            self.goTangent(goal_pos) # Go tangent
        
    def goStraight(self,  goal_pos): 
        print ('GO STRAIGHT') 
        if self.euclidean_distance(self.position, goal_pos) > self.vision_radius:
            const = sqrt(pow(self.vision_radius, 2) / (pow(goal_pos.x - self.position.x, 2) + pow(goal_pos.y - self.position.y, 2)))
            self.waypoint.x =  self.position.x + const * (goal_pos.x - self.position.x)
            self.waypoint.y =  self.position.y + const * (goal_pos.y - self.position.y)
            self.waypoint.z = 0
            tolerance = self.d
            collisionAvoidance = True
        elif self.euclidean_distance(self.position,  goal_pos) > self.d:
            self.waypoint = goal_pos
            tolerance = self.d
            collisionAvoidance = True
        else:
            self.waypoint = goal_pos
            tolerance = self.tolerance
            collisionAvoidance = False
        
        value = self.moveToPoint(self.waypoint,  tolerance,  collisionAvoidance)
        
        self.velocity_publisher.publish(self.stop())
        
        if value == False :
            self.change_state(3)
        
    def goTangent(self,  goal_pos):
        print('GO TANGENT!')
        nextPoint = self.calculateNextWaypoint(goal_pos)
        if nextPoint is not None:
            self.waypoint = nextPoint
            self.turnToPoint(self.waypoint)
            value = self.moveToPoint(self.waypoint,  self.d,  True)
            self.velocity_publisher.publish(self.stop())
            if value == False:
                self.change_state(3)
        else:
            while ( self.collisionScanner > self.d):
               self.velocity_publisher.publish(self.move_forward())
               self.rate.sleep()
            self.velocity_publisher.publish(self.stop())
            self.change_state(1)
                
    # State 1:
    def transition(self,  goal_pos):
        self.dfollow = self.getDFollowed(goal_pos)
        if self.regions['fleft'] < self.regions['fright'] :
            self.region = ['front',  'fleft',  'bleft',  'back',  'bright',  'fright']
            self.edgeScanner =  0
            self.direction = 1
        else:
            self.region = ['front',  'fright',  'bright',  'back',  'bleft',  'fleft']
            self.edgeScanner =  1
            self.direction = -1
            
        self.change_state(2)

    # State 2:
    def followWall(self):
        d = self.d
        if self.regions[self.region[0]] > self.d and self.regions[self.region[1]] > self.d and self.regions[self.region[2]] > self.d :
            #print('Case1: Nothing')
            if self.regions[self.region[0]] >= self.vision_radius and self.regions[self.region[1]] >= self.vision_radius and self.regions[self.region[2]]  > self.vision_radius:
                self.twist_msg = self.stop()
                self.change_state(0)
                self.points = [ ]
            else:
                self.twist_msg = self.move(self.direction)
        elif self.regions[self.region[0]]<= d and self.regions[self.region[1]] <= d and self.regions[self.region[2]]  <= d and self.regions[self.region[3]] <= d and self.regions[self.region[4]] <= d and self.regions[self.region[5]] <=d:
            #print('Case2: Everywhere')
            self.twist_msg = self.stop()
        elif self.regions[self.region[0]] <= d:
            #print('Case3: Front')
            self.twist_msg = self.turn(-1*self.direction)
        elif self.regions[self.region[0]] > d and self.regions[self.region[1]] <= d and self.regions[self.region[2]]  > d:
            #print('Case4: Fleft')
            self.twist_msg = self.move(-1*self.direction)
        elif self.regions[self.region[0]] > d and self.regions[self.region[1]] > d and self.regions[self.region[2]]  <= d:
            #print('Case5: Bleft ')
            self.twist_msg = self.move(self.direction)
        elif self.regions[self.region[0]] > d and self.regions[self.region[1]] <= d and self.regions[self.region[2]]  <= d:
           # print('Case6: Fleft and Bleft')
            if self.edgeScanners[self.edgeScanner] > d:
               # print('Edge')
                self.twist_msg = self.move(self.direction)
            else:
               # print('No Edge')
                self.twist_msg = self.move_forward()
        else:
            self.twist_msg = self.stop()
            print('unknown case')

        if next((x for x in self.points[:-5] if self.euclidean_distance(x, self.position) <= 0.3 ), -1) != -1:
            self.twist_msg = self.stop()
            self.change_state(0)
            self.points = [ ]

        self.velocity_publisher.publish(self.twist_msg)
        
    #State 3:
    def collisionAvoidance(self):
       # pdb.set_trace()
        if self.collisionScanner < self.d:
            obstaclePosition = self.getObstaclePosition()
            if obstaclePosition > 0:
                count = 0
                while(self.collisionScanner <= self.d):
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
                    count+=1
                    if count == 4:
                        self.change_state(0)
                        break
            else:
               self.change_state(1)
        else:
            self.change_state(0)

    #State 4: Goal reached
    def goalReached(self):
        self.heuristicDistance = -1
        self.velocity_publisher.publish(self.stop())
        self.rate.sleep()
    
    def end_procedure(self,  tolerance = 0.2):
        if self.end == 'despawn':
            print('DESPAWN-MODE')
            # is handled by spawning controller
            pass
        elif self.end == 'stay':
              print('STAY-MODE')
              while True:
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
        elif self.end == 'idle':
            print('IDLE-MODE')
            while True:
                if self.frontScanner > self.d:
                    self.velocity_publisher.publish(self.move_forward())
                else:
                    self.velocity_publisher.publish(self.turn_right())
                self.rate.sleep()
        elif self.end == 'start':
            print('START-MODE')
            self.change_state(0)
            while self.euclidean_distance(self.position, self.init_pos) > tolerance:
                    if self.state == 0:
                        self.motionToGoal(self.init_pos)
                    elif self.state == 1:
                        self.transition(self.init_pos)
                    elif self.state == 2:
                        self.followWall()
                    elif self.state == 3:
                        self.collisionAvoidance()
                        
                    self.rate.sleep()
                    
                    if self.state == 2:
                        if self.euclidean_distance(self.point,  self.position) >= 0.1:
                            self.points.append(self.point)
                            self.point = self.position

                        self.dreach = self.euclidean_distance(self.position, self.init_pos)
                        
                        if (self.dreach) < self.dfollow:
                            self.velocity_publisher.publish(self.stop())
                            self.rate.sleep()
                            self.change_state(0)
                            self.points = [ ]
                
            while True:
                self.velocity_publisher.publish(self.stop())
                self.rate.sleep()

###########################THE ALGORITHM########################################
    def tangentBug(self, tolerance = 0.2):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            #Robots has found all goals
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Done! ' % (self.robot_name))
                self.end_procedure()
            #Robot is waiting for a new goal
            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Waiting for a new goal' % (self.robot_name))
            #Robot has received a new goal
            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                if self.init_pos == Point():
                    self.init_pos = self.position
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
                        self.motionToGoal(self.goal_pos)
                    elif self.state == 1:
                        self.transition(self.goal_pos)
                    elif self.state == 2:
                        self.followWall()
                    elif self.state == 3:
                        self.collisionAvoidance()
                        
                    self.rate.sleep()
                    
                    if self.state == 2:
                        if self.euclidean_distance(self.point,  self.position) >= 0.1:
                            self.points.append(self.point)
                            self.point = self.position

                        self.dreach = self.euclidean_distance(self.position, self.goal_pos)
                        
                        if (self.dreach) < self.dfollow:
                            self.velocity_publisher.publish(self.stop())
                            self.rate.sleep()
                            self.change_state(0)
                            self.points = [ ]

                #Robot has found the current goal
                self.change_state(3)
                self.goalReached()

            self.rate.sleep()

##############################MAIN##############################################

if __name__ == "__main__" :

    try:
        x = TangentBug()
        x.tangentBug()
    except rospy.ROSInterruptException:
        traceback.print_exc()
