#!/usr/bin/env python
import rospy
import traceback

#import pdb
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, fabs, pi,  sin,  cos,  ceil

class Bug2:

    state_dict = {
        0: 'facing the goal',
        1: 'follow the MLine',
        2: 'follow the wall',
        3: 'goal reached'
    }

########INITIALIZATION#############################################

    def __init__(self, rate = 20, vision_radius = 3.5,  max_vel = 0.22,  robot_length = 0.2):
        
        #Initialize node
        rospy.init_node('go_to_goal', anonymous=True)
        
        #Initialize publisher
        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)
        
        #Initialize subscriber
        self.pos_subscriber = rospy.Subscriber('odom',
                                                Odometry, self.callback_odom)

        self.finished_subscriber = rospy.Subscriber('benchmark/finished',
                                                Bool, self.callback_finished)

        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.callback_laser)

        self.goal_subscriber = rospy.Subscriber('benchmark/waypoint', Point, self.callback_target)
        
        #Initialize Variables
        self.end = rospy.get_param("/end_procedure")
        self.vision_radius = vision_radius
        self.max_vel = max_vel
        self.robot_length = robot_length
        self.rate = rospy.Rate(rate)
        self.start_pos = Point()
        self.init_pos = Point()
        self.goal_pos = Point()
        self.position = Point()
        self.wall_hit_point = Point()
        self.point = Point()
        self.leavePoint = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.orientation = 0
        self.state = 0
        self.scanner = []
        self.points =[]
        self.max_step = 0.055
        self.d = self.robot_length + 2 * self.max_step + 0.1
        self.FrontScannerSize = int(ceil(((self.robot_length +0.2) * 360) /( 2* pi * self.d)))
        if self.isEven(self.FrontScannerSize) != True :
            self.FrontScannerSize += 1
        
       
########CALLBACKS###############################################
    def callback_target(self, data):
        self.goal_pos = data
       
    def callback_finished(self, data):
        self.finished = data
        
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
        self.edgeScanner =  min(min(data.ranges[30:60]),self.vision_radius)
        self.frontScanner =  min(min(data.ranges[0:(self.FrontScannerSize/2)]),min(data.ranges[(360 - self.FrontScannerSize / 2):360]),self.vision_radius)
            
    def callback_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta
        
########HELPER-METHODS###########################################

    # Changes the robots state
    def change_state(self, state):
        if state is not self.state:
            if state == 0:
                self.leavePoint = self.position
            print ('State %s: %s' % (state, self.state_dict[state]))
            self.state = state
    
    def isEven(self, number):
        mod = number % 2
        if mod > 0:
            return False
        else:
            return True

    # Angle error for the P-Controller and other computations
    def angle_error(self,  goal):
        goal_angle = atan2(goal.y - self.position.y, goal.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = atan2(sin(angle), cos(angle))
        return angle

    # The distance the robot has moved when it was following the wall of an obstacle
    def moved_distance(self):
        return sqrt(pow(self.position.y - self.wall_hit_point.y, 2) + pow(self.position.x - self.wall_hit_point.x, 2))

    # The robots distance to the MLine
    def line_distance(self,  goal):
        point_1 = self.start_pos
        point_2 = goal
        point_x = self.position
        numerator = fabs((point_2.y - point_1.y) * point_x.x - (point_2.x - point_1.x) * point_x.y + (point_2.x * point_1.y) - (point_2.y * point_1.x))
        denominator = sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2))
        return numerator / denominator

    # The robots distance to the goal
    def euclidean_distance(self,  pos,  goal):
        return sqrt(pow((goal.x - pos.x), 2) + pow((goal.y - pos.y), 2))
    
    #Robot Navigation
    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5
        return msg

    def move_left(self):
        msg = Twist()
        msg.linear.x = self.max_vel
        msg.angular.z = 0.5
        return msg
    
    def move_right(self):
        msg = Twist()
        msg.linear.x = self.max_vel
        msg.angular.z = -0.5
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

########STATE-METHODS############################################

    # State 0: Robot turns until it looks directly to the goal position
    def faceTheGoal(self, goal,  tolerance = 0.01):
        if fabs(self.angle_error(goal)) > tolerance:
            if (self.angle_error(goal) < 0):
                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = -0.5
            else:
                self.twist_msg.linear.x = 0
                self.twist_msg.angular.z = 0.5
        else:
            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = 0
            self.change_state(1)
 
    # State 1: Robot follows the MLine until it reaches the goal or hits an obstacle
    def followMLine(self,  goal):
        if self.frontScanner > self.d:
                self.twist_msg.angular.z = self.angle_error(goal)
                self.twist_msg.linear.x = self.max_vel
        else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.wall_hit_point = self.position
                self.point = self.position
                self.change_state(2)

    def followWall(self,  goal):
        d = self.d
        if self.line_distance(goal) <= 0.1 and self.moved_distance() > self.robot_length and self.euclidean_distance(self.position,  goal) < self.euclidean_distance(self.wall_hit_point,  goal) and self.euclidean_distance(self.leavePoint,  self.position) > self.robot_length:
            self.twist_msg = self.stop()
            self.change_state(0)
            self.points = [ ]
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] > d :
            print('Case1: Nothing')
            if self.regions['front'] >= self.vision_radius and self.regions['fleft'] >= self.vision_radius and self.regions['bleft'] > self.vision_radius:
               self.twist_msg = self.stop()
               self.change_state(0)
               self.points = [ ]
            else:
                self.twist_msg = self.move_left()
        elif self.regions['front'] <= d and self.regions['fleft'] <= d and self.regions['bleft'] <= d and self.regions['back'] <= d and self.regions['bright'] <= d and self.regions['fright'] <=d:
            print('Case2: Everywhere')
            self.twist_msg = self.stop()
        elif self.regions['front'] <= d:
            print('Case3: Front')
            self.twist_msg = self.turn_right()
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] > d:
            print('Case4: Fleft')
            self.twist_msg = self.move_right()
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] <= d:
            print('Case5: Bleft ')
            self.twist_msg = self.move_left()
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] <= d:
            print('Case6: Fleft and Bleft')
            if self.edgeScanner > d:
                print('Edge')
                self.twist_msg = self.move_left()
            else:
                print('No Edge')
                self.twist_msg = self.move_forward()
        else:
            self.twist_msg = self.stop()
            print('unknown case')
    
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
            while self.euclidean_distance(self.position,  self.init_pos) > tolerance:
                if self.state == 0:
                    self.faceTheGoal(self.init_pos)
                elif self.state == 1:
                    self.followMLine(self.init_pos)
                elif self.state == 2:
                    self.followWall(self.init_pos)
                
                if self.state == 2:
                    if next((x for x in self.points[:-5] if self.euclidean_distance(x, self.position) <= 0.3 ), -1) != -1:
                        self.twist_msg = self.stop()
                        self.change_state(0)
                        self.points = [ ]
                        
                self.velocity_publisher.publish(self.twist_msg)
                self.rate.sleep()
                
                if self.state == 2:
                    if self.euclidean_distance(self.point,  self.position) >= 0.1:
                        self.points.append(self.point)
                        self.point = self.position
                
            while True:
                self.velocity_publisher.publish(self.stop())
                self.rate.sleep()
                    
########MAIN-METHOD##############################################

    def bug2(self, tolerance = 0.2):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('Done!')
                self.end_procedure()

            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('Waiting for a new goal')

            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                if self.init_pos == Point():
                    self.init_pos = self.position
                print('Goal received')
                new_goal_pos = self.goal_pos
                self.change_state(0)
                self.start_pos =  self.position

                while self.euclidean_distance(self.position,  self.goal_pos) > tolerance:
                    if self.state == 0:
                        self.faceTheGoal(self.goal_pos)
                    elif self.state == 1:
                        self.followMLine(self.goal_pos)
                    elif self.state == 2:
                        self.followWall(self.goal_pos)
                    
                    if self.state == 2:
                        if next((x for x in self.points[:-5] if self.euclidean_distance(x, self.position) <= 0.3 ), -1) != -1:
                            self.twist_msg = self.stop()
                            self.change_state(0)
                            self.points = [ ]
                            
                    self.velocity_publisher.publish(self.twist_msg)
                    self.rate.sleep()
                    
                    if self.state == 2:
                        if self.euclidean_distance(self.point,  self.position) >= 0.1:
                            self.points.append(self.point)
                            self.point = self.position
                    
                print('Found the goal!')
                self.velocity_publisher.publish(self.stop())

            self.rate.sleep()

if __name__ == "__main__" :
    try:
        x = Bug2()
        x.bug2()
    except rospy.ROSInterruptException:
        traceback.print_exc()
