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

class Bug2:

    state_dict = {
        0: 'facing the goal',
        1: 'follow the MLine',
        2: 'follow the wall',
        3: 'goal reached'
    }

    def __init__(self, rate = 20, vision_radius = 3.5):

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
        self.new_goal_pos = Point()
        self.twist_msg = Twist()
        self.start_pos = Point()
        self.finished = Bool()

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

    # Angle error for the P-Controller
    def angle_error(self):
        goal_angle = atan2(self.new_goal_pos.y - self.position.y, self.new_goal_pos.x - self.position.x)
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

    # The distance the robot has moved when it was following the wall of an obstacle
    def moved_distance(self):
        return sqrt(pow(self.position.y - self.wall_hit_point.y, 2) + pow(self.position.x - self.wall_hit_point.x, 2))

    # The robots distance to the MLine
    def line_distance(self):
        point_1 = self.start_pos
        point_2 = self.new_goal_pos
        point_x = self.position

        numerator = fabs((point_2.y - point_1.y) * point_x.x - (point_2.x - point_1.x) * point_x.y + (point_2.x * point_1.y) - (point_2.y * point_1.x))
        denominator = sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2))

        return numerator / denominator

    # The robots distance to the goal
    def euclidean_distance(self):
        return sqrt(pow((self.new_goal_pos.x - self.position.x), 2) + pow((self.new_goal_pos.y - self.position.y), 2))

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
        while not rospy.is_shutdown():
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Done! ' % (self.robot_name))

            elif self.finished == Bool(False) and self.new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('[%s] - Waiting for a new goal' % (self.robot_name))

            elif self.finished == Bool(False) and self.new_goal_pos != self.goal_pos:
                print('[%s] - Goal received' % (self.robot_name))
                self.new_goal_pos = self.goal_pos
                self.change_state(0)
                self.start_pos =  self.position

                while self.euclidean_distance() > tolerance:
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
