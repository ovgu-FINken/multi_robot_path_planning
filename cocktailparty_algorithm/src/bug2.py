#!/usr/bin/env python
import rospy
import traceback

#import pdb
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
        rospy.init_node('bug2', anonymous=True)

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
        self.wp_threshold = rospy.get_param("/wp_threshold")
        self.vision_radius = rospy.get_param("~vision_radius")
        self.max_vel = rospy.get_param("~max_vel")
        self.robot_size = rospy.get_param("~robot_size")
        rate =rospy.get_param("~rate")
        self.safety_distance = rospy.get_param("~safety_distance")
        self.collision_scanner_size =  rospy.get_param("~collision_scanner_size")
        self.rate = rospy.Rate(rate)
        self.start_pos = Point()
        self.init_pos = Point()
        self.goal_pos = Point()
        self.position = Point()
        self.wall_hit_point = Point()
        self.last_leave_point = Point()
        self.past_pos = Point()
        self.leavePoint = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.orientation = 0
        self.state = 0
        self.scanner = []
        self.past_positions =[]

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
        self.edge_scanner =  min(min(data.ranges[30:60]),self.vision_radius)
        self.collision_scanner =  min(min(data.ranges[0:(self.collision_scanner_size/2)]),min(data.ranges[(360 - self.collision_scanner_size / 2):360]),self.vision_radius)

    def callback_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta

########HELPER-METHODS###########################################
    # Angle error for the P-Controller and other computations
    def angle_error(self,  goal):
        goal_angle = atan2(goal.y - self.position.y, goal.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = atan2(sin(angle), cos(angle))
        return angle
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

########STATE-METHODS AND END-PROCEDURE############################################
   
   # Changes the robots state
    def change_state(self, state):
        if state is not self.state:
            print ('State %s: %s' % (state, self.state_dict[state]))
            self.state = state
            
    # State 0: Robot turns until it looks directly to the goal position
    def face_the_goal(self, goal,  tolerance = 0.01):
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
    def follow_mline(self,  goal):
        if self.collision_scanner > self.safety_distance:
                self.twist_msg.angular.z = self.angle_error(goal)
                self.twist_msg.linear.x = self.max_vel
        else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.wall_hit_point = self.position
                self.past_pos = self.position
                self.change_state(2)

    def follow_wall(self,  goal):
        d = self.safety_distance
        #General leaving condition
        if self.line_distance(goal) <= 0.1 and self.euclidean_distance(self.position,  self.wall_hit_point) > self.robot_size and self.euclidean_distance(self.position,  goal) < self.euclidean_distance(self.wall_hit_point,  goal) and self.euclidean_distance(self.last_leave_point,  self.position) > self.robot_size:
            self.last_leave_point = self.position
            self.twist_msg = self.stop()
            self.change_state(0)
            self.points = [ ]
        #Infinity cycles
        elif next((x for x in self.past_positions[:-1]  if  self.euclidean_distance(x, self.position) <= 0.01) , -1) != -1:
            print ('self.euclidean_distance: %s' % (self.euclidean_distance(self.past_positions[0],  self.position)))
            self.last_leave_point = self.position
            self.velocity_publisher.publish(self.stop())
            self.change_state(0)
            self.past_positions= []
        #Case1: No obstacle
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] > d :
            #Case 1.1: Lost wall
            if self.regions['front'] >= self.vision_radius and self.regions['fleft'] >= self.vision_radius and self.regions['bleft'] > self.vision_radius:
               self.twist_msg = self.stop()
               self.last_leave_point = self.position
               self.change_state(0)
               self.past_positions = [ ]
            #Case 1.2: Distance to wall bigger than safety distance
            else:
                self.twist_msg = self.move_left()
        #Case 2: Obstacle everywhere
        elif self.regions['front'] <= d and self.regions['fleft'] <= d and self.regions['bleft'] <= d and self.regions['back'] <= d and self.regions['bright'] <= d and self.regions['fright'] <=d:
            self.twist_msg = self.stop()
        #Case 3: Wall in front
        elif self.regions['front'] <= d:
            self.twist_msg = self.turn_right()
        #Case 4: Obstacle on the front left side
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] > d:
            self.twist_msg = self.move_right()
        # Case 5: Obstacle on the back left side
        elif self.regions['front'] > d and self.regions['fleft'] > d and self.regions['bleft'] <= d:
            self.twist_msg = self.move_left()
        # Case 6: Obstacle on the front left and back left
        elif self.regions['front'] > d and self.regions['fleft'] <= d and self.regions['bleft'] <= d:
           # Case 6.1: There is an edge upcoming
            if self.edge_scanner > d:
                self.twist_msg = self.move_left()
            # Case 6.2: No edge
            else:
                self.twist_msg = self.move_forward()
        #Case 7: Unknown Casw
        else:
            self.twist_msg = self.stop()
            print('unknown case')

    def end_procedure(self):
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
                if self.collision_scanner > self.safety_distance:
                    self.velocity_publisher.publish(self.move_forward())
                else:
                    self.velocity_publisher.publish(self.turn_right())
                self.rate.sleep()
        elif self.end == 'start':
            print('START-MODE')
            self.change_state(0)
            while self.euclidean_distance(self.position,  self.init_pos) > self.wp_threshold:
                if self.state == 0:
                    self.face_the_goal(self.init_pos)
                elif self.state == 1:
                    self.follow_mline(self.init_pos)
                elif self.state == 2:
                    self.follow_wall(self.init_pos)
                    if self.euclidean_distance(self.past_pos,  self.position) >= 0.1:
                        self.past_positions.append(self.past_pos)
                        self.past_pos = self.position

                self.velocity_publisher.publish(self.twist_msg)
                self.rate.sleep()
                
            while True:
                self.velocity_publisher.publish(self.stop())
                self.rate.sleep()

########MAIN-LOOP#################################################################

    def bug2(self):
        new_goal_pos = Point()
        while not rospy.is_shutdown():
            #Robots has found all goals
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                print('Done!')
                self.end_procedure()
            #Robot is waiting for a new goal
            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('Waiting for a new goal')
            #Robot has received a new goal
            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                if self.init_pos == Point():
                    self.init_pos = self.position
                print('Goal received')
                new_goal_pos = self.goal_pos
                self.change_state(0)
                self.start_pos =  self.position
                
                #Robot is driving towards the goal
                while self.euclidean_distance(self.position,  self.goal_pos) > self.wp_threshold:
                    if self.state == 0:
                        self.face_the_goal(self.goal_pos)
                    elif self.state == 1:
                        self.follow_mline(self.goal_pos)
                    elif self.state == 2:
                        self.follow_wall(self.goal_pos)
                        if self.euclidean_distance(self.past_pos,  self.position) >= 0.1:
                            self.past_positions.append(self.past_pos)
                            self.past_pos = self.position

                    self.velocity_publisher.publish(self.twist_msg)
                    self.rate.sleep()
                #Robot has found the current goal
                print('Found the goal!')
                self.velocity_publisher.publish(self.stop())

            self.rate.sleep()
###################MAIN####################################################

if __name__ == "__main__" :
    try:
        x = Bug2()
        x.bug2()
    except rospy.ROSInterruptException:
        traceback.print_exc()
