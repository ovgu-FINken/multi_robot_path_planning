#!/usr/bin/env python
import rospy
import traceback
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, fabs, pi,  isnan,  degrees,  ceil,  sin,  cos

class Bug2:

    state_dict = {
        0: 'facing the goal',
        1: 'follow the MLine',
        2: 'follow the wall',
        3: 'goal reached'
    }

    def __init__(self, rate = 20, vision_radius = 3.5,  max_vel = 0.22,  robot_length = 0.2):

        rospy.init_node('go_to_goal', anonymous=True)

        self.robot_name = rospy.get_param('~robot_name')

        self.velocity_publisher = rospy.Publisher('cmd_vel',
                                                  Twist, queue_size=10)

        self.pos_subscriber = rospy.Subscriber('odom',
                                                Odometry, self.update_odom)

        self.finished_subscriber = rospy.Subscriber('benchmark/finished',
                                                Bool, self.callback_finished)

        self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.update_laser)

        self.goal_subscriber = rospy.Subscriber('benchmark/waypoint', Point, self.callback_target)

        self.vision_radius = vision_radius
        self.rate = rospy.Rate(rate)
        self.position = Point()
        self.wall_hit_point = Point()
        self.orientation = 0
        self.state = 0
        self.goal_pos = Point()
        self.twist_msg = Twist()
        self.start_pos = Point()
        self.finished = Bool()
        self.max_vel = max_vel
        self.max_step = self.max_vel * (1.0/rate)
        self.scanner = []
        self.robot_length = robot_length

    def callback_target(self, data):
        self.goal_pos = data

    def callback_finished(self, data):
        self.finished = data

    # Changes the robots state
    def change_state(self, state):
        if state is not self.state:
            print ('[%s] - State %s: %s' % (self.robot_name, state, self.state_dict[state]))
            self.state = state

    # Updates the robots current position and orientation
    def update_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta

    # Angle error for the P-Controller and other computations
    def angle_error(self,  goal):
        goal_angle = atan2(goal.y - self.position.y, goal.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = atan2(sin(angle), cos(angle))
        return angle

    # Callback for the laser scanner 
    def update_laser(self, data):
        self.scanner = data.ranges[:]
        self.regions ={
            'front' : min(min(data.ranges[0:30]),min(data.ranges[330:360]),self.vision_radius),
            'fleft' : min(min(data.ranges[30:90]),self.vision_radius),
            'bleft' : min(min(data.ranges[90:150]),self.vision_radius),
            'back' : min(min(data.ranges[150:210]),self.vision_radius),
            'bright' : min(min(data.ranges[210:270]),self.vision_radius), 
            'fright' : min(min(data.ranges[270:330]),self.vision_radius)
            }

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

        self.velocity_publisher.publish(self.twist_msg)

    # 3 functions for the wall following part

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5
        return msg

    def move_left(self,  z):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z =  z
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
        
    # State 1: Robot follows the MLine until it reaches the goal or hits an obstacle
    def followMLine(self):
        d_min = min(self.scanner[:]) - self.robot_length - 0.1
        #Object Oi appears within the robot's range of sensing
#        if  d_min  < self.vision_radius:
            # The collision front is E = phi
        if d_min > 2.0 * self.max_step:
            self.twist_msg.angular.z = self.angle_error(self.goal_pos)
            self.twist_msg.linear.x = self.max_vel
            #Check if next intendet position of robot R is inside or outside of E
        elif self.regions['front'] - self.robot_length - 0.1 > 2 * self.max_step:
                self.twist_msg.angular.z = self.angle_error(self.goal_pos)
                self.twist_msg.linear.x = self.max_vel
        else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.wall_hit_point = self.position
                self.change_state(2)
        # No Object(s) Oi appears within the robot's range of sensing           
#        else:
#            self.twist_msg.angular.z = self.angle_error(self.goal_pos)
#            self.twist_msg.linear.x = self.max_vel
            
        self.velocity_publisher.publish(self.twist_msg)
       
    # State 2: Robot follows the wall of an obstacle until it reaches the leaving point
    # d = collisionFront, Important: greater velocity needs higher collisionFront
    def followWall(self):
        
        d = self.robot_length + 2 * self.max_step + 0.1

        if self.line_distance() < 0.1 and self.moved_distance() > self.robot_length and self.euclidean_distance(self.position,  self.goal_pos) < self.euclidean_distance(self.wall_hit_point,  self.goal_pos) :
            self.twist_msg = self.stop()
            self.change_state(0)

        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] > d :
            print('Follow Wall - Case1: Nothing')
            self.twist_msg = self.stop()
            self.change_state(0)
        elif self.regions['front'] < d:
            print('Follow Wall - Case2: front')
            self.twist_msg = self.turn_right()
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['bleft'] > d:
            print('Follow wall - Case3: fleft')
            self.twist_msg = self.move_left(0.1)
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] < d:
            print('Follow wall - Case4: bleft ')
            self.twist_msg = self.move_left(0.3)
        elif self.regions['front'] > d and self.regions['fleft'] < d and self.regions['bleft'] < d:
            print('Follow wall - Case5: fleft and bleft')
            self.twist_msg = self.move_forward()
        else:
            self.twist_msg = self.stop()
            print('unknown case')

        self.velocity_publisher.publish(self.twist_msg)

    # The path planning algorithm
    def bug2(self, tolerance = 0.1):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Done! ' % (self.robot_name))

            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Waiting for a new goal' % (self.robot_name))

            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                print('[%s] - Goal received' % (self.robot_name))
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
                    self.rate.sleep()

                print('[%s] - Found the goal!' % (self.robot_name))
                self.velocity_publisher.publish(self.stop())

            self.rate.sleep()

if __name__ == "__main__" :
    try:
        x = Bug2()
        x.bug2()
    except rospy.ROSInterruptException:
        traceback.print_exc()
