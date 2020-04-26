#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi

#TODO: Adjust Parameter!! Make it work for multiple robots.
class Bot:

    def __init__(self):

        rospy.init_node('go_to_goal', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/tb3_1/cmd_vel',
                                                  Twist, queue_size=10)

        self.pos_subscriber = rospy.Subscriber('/tb3_1/odom',
                                                Odometry, self.update_odom)

        self.scan_subscriber = rospy.Subscriber('/tb3_1/scan', LaserScan, self.update_laser)

        self.position = Point()
        self.orientation = 0
        self.rate = rospy.Rate(10)
        self.state = "FaceTheGoal"
        self.goal_pos = Point()
        self.goal_pos.x = input("X-Koordinate: ")
        self.goal_pos.y = input("Y-Koordinate: ")
        self.twist_msg = Twist()
        self.initPos = Point()

    def update_odom(self, data):

        self.position = data.pose.pose.position

        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta

    def angle_error(self):
        goal_angle = atan2(self.goal_pos.y - self.position.y, self.goal_pos.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    def obstacle_distance_error(self, tolerance = 0.3):
        return tolerance - self.scanner['fright']

    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = angle - (2 * pi * angle) / (fabs(angle))
        return angle

    def update_laser(self, data):
        self.scanner = {
        'front' : min(min(data.ranges[0:35]),min(data.ranges[325:359]),3.5),
		'fleft' : min(min(data.ranges[36:107]),3.5),
		'left' : min(min(data.ranges[108:179]),3.5),
		'right' : min(min(data.ranges[180:251]),3.5),
		'fright' : min(min(data.ranges[252:324]),3.5)
    }

    def faceTheGoal(self, tolerance = 0.01):
        print("Facing the goal")

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
            self.state = "Follow_MLine"

        self.velocity_publisher.publish(self.twist_msg)

    def followMLine(self):
        print("Following the MLine")

        if self.scanner['front'] < 0.3:
            self.twist_msg.angular.z = 0
            self.twist_msg.linear.x = 0
            self.wall_hit_point = self.position
            self.state = "Follow_Wall"
        else:
            self.twist_msg.angular.z = 2 * self.angle_error()
            self.twist_msg.linear.x = 0.25

        self.velocity_publisher.publish(self.twist_msg)

    def followWall(self):
        print("Following the Wall")
        if self.line_distance() < 0.1 and self.moved_distance() > 0.5:
            print("Juhu raus hier!!")
            self.state = "FaceTheGoal"
        elif self.scanner['front'] < 0.3:
            print("Hindernis vorn!")
            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = 0.5
        else:
            print("Fahren")
            self.twist_msg.angular.z = 4 * self.normalize(self.obstacle_distance_error())
            self.twist_msg.linear.x = 0.1

        self.velocity_publisher.publish(self.twist_msg)

    def moved_distance(self):
        return sqrt(pow(self.position.y - self.wall_hit_point.y, 2) + pow(self.position.x - self.wall_hit_point.x, 2))

    def line_distance(self):
        point_1 = self.initPos
        point_2 = self.goal_pos
        point_x = self.position

        numerator = fabs((point_2.y - point_1.y) * point_x.x - (point_2.x - point_1.x) * point_x.y + (point_2.x * point_1.y) - (point_2.y * point_1.x))
        denominator = sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2))

        return numerator / denominator

    def euclidean_distance(self):
        return sqrt(pow((self.goal_pos.x - self.position.x), 2) + pow((self.goal_pos.y - self.position.y), 2))

    def bug2(self, tolerance = 0.1):
        self.initPos = self.position
        while self.euclidean_distance() > tolerance:
            if self.state == "FaceTheGoal":
                self.faceTheGoal()
            if self.state == "Follow_Wall":
                self.followWall()
            elif self.state == "Follow_MLine":
                self.followMLine()
            self.rate.sleep()

        print("Found the goal!!")
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.velocity_publisher.publish(self.twist_msg)

if __name__ == '__main__':
    try:
        x = Bot()
        x.bug2()
    except rospy.ROSInterruptException:
        pass
