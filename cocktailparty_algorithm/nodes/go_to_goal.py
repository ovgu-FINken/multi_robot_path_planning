#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi

#TODO: Adjust Parameter!! Make it work for multiple robots.
class Bot:

    state_dict = {
        0: 'facing the goal',
        1: 'follow the MLine',
        2: 'follow the wall',
        3: 'goal reached'
    }

    def __init__(self):

        rospy.init_node('go_to_goal', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/tb3_1/cmd_vel',
                                                  Twist, queue_size=10)

        self.pos_subscriber = rospy.Subscriber('/tb3_1/odom',
                                                Odometry, self.update_odom)

        self.scan_subscriber = rospy.Subscriber('/tb3_1/scan', LaserScan, self.update_laser)

        self.rate = rospy.Rate(10)
        self.position = Point()
        self.wall_hit_point = Point()
        self.orientation = 0
        self.state = 0
        self.goal_pos = Point()
        self.goal_pos.x = input("X-Koordinate: ")
        self.goal_pos.y = input("Y-Koordinate: ")
        self.twist_msg = Twist()
        self.initPos = Point()

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
        'front' : min(min(data.ranges[0:35]),min(data.ranges[325:359]),3.5),
		'fleft' : min(min(data.ranges[36:107]),3.5),
		'left' : min(min(data.ranges[108:179]),3.5),
		'right' : min(min(data.ranges[180:251]),3.5),
		'fright' : min(min(data.ranges[252:324]),3.5)
    }

    # The distance the robot has moved when it was following the wall of an abstacle
    def moved_distance(self):
        return sqrt(pow(self.position.y - self.wall_hit_point.y, 2) + pow(self.position.x - self.wall_hit_point.x, 2))

    # The robots distance to the MLine
    def line_distance(self):
        point_1 = self.initPos
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
        self.initPos = self.position
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

if __name__ == '__main__':
    try:
        x = Bot()
        x.bug2()
    except rospy.ROSInterruptException:
        pass
