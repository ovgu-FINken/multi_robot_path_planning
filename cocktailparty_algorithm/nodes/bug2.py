#!/usr/bin/env python
import rospy
import traceback
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, fabs, pi,  sin,  cos

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
        self.vision_radius = vision_radius
        self.max_vel = max_vel
        self.robot_length = robot_length
        self.rate = rospy.Rate(rate)
        self.start_pos = Point()
        self.goal_pos = Point()
        self.position = Point()
        self.wall_hit_point = Point()
        self.point = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.orientation = 0
        self.state = 0
        self.scanner = []
        self.points =[]
        self.max_step = 0.055

########CALLBACKS###############################################
    def callback_target(self, data):
        self.goal_pos = data

    def callback_finished(self, data):
        self.finished = data
        
    def callback_laser(self, data):
        self.scanner = data.ranges[:]
        self.regions ={
            'front' : min(min(data.ranges[0:36]),min(data.ranges[324:360]),self.vision_radius),
            'fleft' : min(min(data.ranges[36:90]),self.vision_radius),
            'bleft' : min(min(data.ranges[90:144]),self.vision_radius),
            'back' : min(min(data.ranges[144:216]),self.vision_radius),
            'bright' : min(min(data.ranges[216:270]),self.vision_radius), 
            'fright' : min(min(data.ranges[270:324]),self.vision_radius)
            }
            
    def callback_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta
        
########HELPER-METHODS###########################################

    # Changes the robots state
    def change_state(self, state):
        if state is not self.state:
            print ('State %s: %s' % (state, self.state_dict[state]))
            self.state = state

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
    def line_distance(self):
        point_1 = self.start_pos
        point_2 = self.goal_pos
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
        msg.linear.x = 0.1
        msg.angular.z = 0.3
        return msg

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.1
        return msg

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        return msg

########STATE-METHODS############################################

    # State 0: Robot turns until it looks directly to the goal position
    def faceTheGoal(self, tolerance = 0.01):
        if fabs(self.angle_error(self.goal_pos)) > tolerance:
            if (self.angle_error(self.goal_pos) < 0):
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
    def followMLine(self):
        if min(self.regions['front'],  self.regions['fright'],  self.regions['fleft']) - self.robot_length - 0.1  > 2 * self.max_step:
                self.twist_msg.angular.z = self.angle_error(self.goal_pos)
                self.twist_msg.linear.x = self.max_vel
        else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.wall_hit_point = self.position
                self.point = self.position
                self.change_state(2)

    def followWall(self):
        d = self.robot_length + 2 * self.max_step + 0.1

        if self.line_distance() <= 0.1 and self.moved_distance() > self.robot_length and self.euclidean_distance(self.position,  self.goal_pos) < self.euclidean_distance(self.wall_hit_point,  self.goal_pos) :
            self.twist_msg = self.stop()
            self.change_state(0)
            self.points = [ ]
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] > d :
            print('Follow Wall - Case1: Nothing')
            if self.regions['front'] >= self.vision_radius and self.regions['fleft'] >= self.vision_radius and self.regions['bleft'] > self.vision_radius:
               self.twist_msg = self.stop()
               self.change_state(0)
               self.points = [ ]
            else:
                self.twist_msg = self.move_left()
        elif self.regions['front'] <= d and self.regions['fleft'] <= d and self.regions['bleft'] <= d and self.regions['back'] <= d and self.regions['bright'] <= d and self.regions['fright'] <=d:
            self.twist_msg = self.stop()
        elif self.regions['front'] <= d:
            print('Follow Wall - Case2: front')
            self.twist_msg = self.turn_right()
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] > d:
            print('Follow wall - Case3: fleft')
            self.twist_msg = self.move_left()
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] <= d:
            print('Follow wall - Case4: bleft ')
            self.twist_msg = self.move_left()
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] <= d:
            print('Follow wall - Case5: fleft and bleft')
            self.twist_msg = self.move_forward()
        else:
            self.twist_msg = self.stop()
            print('unknown case')
        
########MAIN-METHOD##############################################

    def bug2(self, tolerance = 0.1):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('Done!')

            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('Waiting for a new goal')

            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                print('Goal received')
                new_goal_pos = self.goal_pos
                self.change_state(0)
                self.start_pos =  self.position

                while self.euclidean_distance(self.position,  self.goal_pos) > tolerance:
                    if self.state == 0:
                        self.faceTheGoal()
                    elif self.state == 1:
                        self.followMLine()
                    elif self.state == 2:
                        self.followWall()
                    
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
