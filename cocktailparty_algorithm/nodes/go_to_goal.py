#!/usr/bin/env python
import rospy
import sys
import traceback
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi

#TODO: Adjust Parameter!! Make it work for multiple robots.
class Move:

    state_dict = {
        0: 'facing the goal',
        1: 'follow the MLine',
        2: 'follow the wall',
        3: 'goal reached'
    }

    # Important: Step size (step_size) of robot has to be smaller than the radius of vision (vision_radius)
    # minus the step upper bound (step_max) of each robot.

    # With: rate = 20Hz, max_linear_velocity = 0.25m/s, step_max = 0,0125m, vision_radius = 3.5m
    # s <= 3.5 - 0,0125 at every cycle

    def __init__(self, rate = 20, vision_radius = 3.5):

        rospy.init_node('go_to_goal', anonymous=True)

        self.robot_name = rospy.get_param('~robot_name')

        self.velocity_publisher = rospy.Publisher(self.robot_name +  '/cmd_vel',
                                                  Twist, queue_size=10)

        self.pos_subscriber = rospy.Subscriber(self.robot_name + '/odom',
                                                Odometry, self.update_odom)

        self.scan_subscriber = rospy.Subscriber(self.robot_name + '/scan', LaserScan, self.update_laser)

        #### BENCHMARK ####
        self.start_pos_subscriber = rospy.Subscriber(self.robot_name + '/benchmark/start_pos', Point, self.callback_start_pos)
        self.goal_subscriber = rospy.Subscriber(self.robot_name + '/benchmark/waypoint', Point, self.callback_target)
        self.finished_subscriber = rospy.Subscriber(self.robot_name + '/benchmark/finished', Bool, self.callback_finished)

        #### BENCHMARK ####

        self.rate = rospy.Rate(rate)
        self.vision_radius = vision_radius
        self.position = Point()
        self.wall_hit_point = Point()
        self.orientation = 0
        self.state = 0
        self.goal_pos = Point()
        self.twist_msg = Twist()
        self.start_pos = Point()

    ############ BENCHMARK ############

        self.finished = Bool()
        self.flag_start = False
        self.flag_goal = False

    def callback_target(self, data):
        self.goal_pos = data
        self.flag_goal = True

    def callback_start_pos(self, data):
        self.start_pos = [data.x, data.y, data.z]
        #self.flag_start = True

    def callback_finished(self, data):
        self.finished = data

    # def wait_for_pos(self):
    #     while self.flag_start == False:
    #         rospy.Rate(0.5).sleep()
    #     rospy.loginfo("["+self.robot_name+"]: Start positions received!")

    def wait_for_targets(self):
        while self.flag_goal == False:
            rospy.loginfo("["+self.robot_name+"]: Waiting for target point")
            rospy.Rate(0.5).sleep()

    ############ BENCHMARK ############

    # Changes the robots state
    def change_state(self, state):
        if state is not self.state:
            print ('[%s] - %s' % (state, self.state_dict[state]))
            self.state = state

    # Updates the robots current position and orientation
    def update_odom(self, data):

        self.position = data.pose.pose.position

        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta

    # Angle error for the P-Controller
    def angle_error(self):
        goal_angle = atan2(self.goal_pos.y - self.position.y, self.goal_pos.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = angle - (2 * pi * angle) / (fabs(angle))
        return angle

    # Callback for the laser scanner and defining different regions
    def update_laser(self, data):
        self.scanner = {
        'front' : min(min(data.ranges[0:35]),min(data.ranges[325:359]),self.vision_radius),
		'fleft' : min(min(data.ranges[36:107]),self.vision_radius),
		'left' : min(min(data.ranges[108:179]),self.vision_radius),
		'right' : min(min(data.ranges[180:251]),self.vision_radius),
		'fright' : min(min(data.ranges[252:324]),self.vision_radius)
    }

    # The distance the robot has moved when it was following the wall of an abstacle
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
    def euclidean_distance(self):
        return sqrt(pow((self.goal_pos.x - self.position.x), 2) + pow((self.goal_pos.y - self.position.y), 2))


    # State 0: Robot turns until it looks directly to the goal position
    def faceTheGoal(self, tolerance = 0.01):

        if fabs(self.angle_error()) > tolerance:
            if (self.angle_error() < 0):
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
        msg.angular.z = -0.3
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

    # Stops the robot
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        return msg

    # State 1: Robot follows the MLine until it reaches the goal or hits an obstacle
    def followMLine(self):
        if self.scanner['front'] < 0.3:
            self.twist_msg.angular.z = 0
            self.twist_msg.linear.x = 0
            self.wall_hit_point = self.position
            self.change_state(2)
        else:
            self.twist_msg.angular.z = 2 * self.angle_error()
            self.twist_msg.linear.x = 0.25

        self.velocity_publisher.publish(self.twist_msg)

    # State 2: Robot follows the wall of an obstacle until it reaches the leaving point
    # d = collisionFront, Important: greater velocity needs higher collisionFront
    def followWall(self, d = 0.3):

        if self.line_distance() < 0.1 and self.moved_distance() > 0.5:
            self.change_state(0)

        elif self.scanner['front'] > d and self.scanner['fleft'] > d and self.scanner['fright'] > d and self.scanner['left'] > d:
            #print('Follow Wall - Case1: Nothing')
            self.twist_msg = self.move_left()
        elif self.scanner['front'] < d and self.scanner['fleft'] > d and self.scanner['fright'] > d and self.scanner['left'] > d:
            #print('Follow Wall - Case2: front')
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] > d and self.scanner['fleft'] < d and self.scanner['fright'] > d and self.scanner['left'] > d:
            #print('Follow wall - Case3: fleft')
            self.twist_msg = self.move_forward()
        elif self.scanner['front'] > d and self.scanner['fleft'] < d and self.scanner['fright'] > d and self.scanner['left'] < d:
            #print('Follow wall - Case4: fleft and left')
            self.twist_msg = self.move_left()
        elif self.scanner['front'] > d and self.scanner['fleft'] > d and self.scanner['fright'] < d:
            #print('Follow wall - Case5: fright')
            self.twist_msg = self.move_left()
        elif self.scanner['front'] < d and self.scanner['fleft'] < d and self.scanner['fright'] > d and self.scanner['left'] > d:
            #print('Follow wall - Case6: front and fleft )
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] < d and self.scanner['fleft'] < d and self.scanner['fright'] > d and self.scanner['left'] < d:
            #print('Follow wall - Case7: front, fleft and left')
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] < d and self.scanner['fleft'] > d and self.scanner['fright'] < d:
            #print('Follow wall - Case8: front and fright')
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] < d and self.scanner['fleft'] < d and self.scanner['fright'] < d and self.scanner['left'] < d:
            #print('Follow wall - Case9: front, fleft, fright and left')
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] < d and self.scanner['fleft'] < d and self.scanner['fright'] < d and self.scanner['left'] > d:
            #print('Follow wall - Case10: front, fleft, fright')
            self.twist_msg = self.turn_right()
        elif self.scanner['front'] > d and self.scanner['fleft'] < d and self.scanner['fright'] < d:
            #print('Follow wall - Case11: fleft, fright')
            self.twist_msg = self.move_left()
        else:
            self.twist_msg = self.stop
            print('unknown case')
            rospy.loginf(scanner)

        self.velocity_publisher.publish(self.twist_msg)

    # The path planning algorithm
    def bug2(self, tolerance = 0.1):
        self.wait_for_pos()
        while not rospy.is_shutdown():
            while self.finished == Bool(False):
                self.wait_for_targets()
                self.change_state(0)
                while self.euclidean_distance() > tolerance:
                    if self.state == 0:
                        self.faceTheGoal()
                    elif self.state == 1:
                        self.followMLine()
                    elif self.state == 2:
                        self.followWall()
                        self.rate.sleep()

                self.change_state(3)
                self.velocity_publisher.publish(self.stop())
                self.flag_goal == false
        self.velocity_publisher.publish(self.stop())

if __name__ == "__main__" :
        x = Move()
        x.bug2()
