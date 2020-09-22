#!/usr/bin/env python
import rospy
import traceback

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
    }
 
########INITIALIZATION#############################################

    def __init__(self, rate = 20, max_vel = 0.22,  robot_size = 0.2):
        
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
        self.end_scenario = rospy.get_param("/end_procedure")
        self.wp_threshold = rospy.get_param("/wp_threshold")
        self.max_vel = max_vel
        self.robot_size = robot_size
        self.rate = rospy.Rate(rate)
        self.start_pos = Point()
        self.init_pos = Point()
        self.goal_pos = Point()
        self.position = Point()
        self.wall_hit_point = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.orientation = 0
        self.state = 0
        self.scanner_regions = []
        self.past_positions =[]
        self.max_step = 0.055
        self.vision_radius = 3.5
        self.safety_distance = self.robot_size + 2 * self.max_step + 0.1
        self.front_scanner_size = self.calc_front_scanner_size()
        
########CALLBACKS###############################################
    def callback_target(self, data):
        self.goal_pos = data
       
    def callback_finished(self, data):
        self.finished = data
        
    def callback_laser(self, data):
        self.scanner = data.ranges[:]
        self.scanner_regions ={
            'front' : min(min(data.ranges[0:30]),min(data.ranges[330:360]),self.vision_radius),
            'fleft' : min(min(data.ranges[30:90]),self.vision_radius),
            'bleft' : min(min(data.ranges[90:150]),self.vision_radius),
            'back' : min(min(data.ranges[150:210]),self.vision_radius),
            'bright' : min(min(data.ranges[210:270]),self.vision_radius), 
            'fright' : min(min(data.ranges[270:330]),self.vision_radius)
            }
        self.edge_scanner =  min(min(data.ranges[30:60]),self.vision_radius)
        self.front_scanner =  min(min(data.ranges[0:(self.front_scanner_size/2)]),min(data.ranges[(360 - self.front_scanner_size / 2):360]),self.vision_radius)
            
    def callback_odom(self, data):
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion ([orientation.x, orientation.y, orientation.z, orientation.w])
        self.orientation = theta
        
########HELPER-METHODS###########################################
  
    def change_state(self, state):
        if state is not self.state:
            print ('State %s: %s' % (state, self.state_dict[state]))
            self.state = state
   
   # Calculates size of the robots front scanner
    def calc_front_scanner_size(self):
        scanner_size = int(ceil(((self.robot_size +0.2) * 360) /( 2* pi * self.safety_distance)))
        if self.is_even(scanner_size) != True:
            scanner_size += 1
        return scanner_size
        
    # Checks if a number is even 
    def is_even(self, number):
        mod = number % 2
        if mod > 0:
            return False
        else:
            return True
            
    # Angle error for the P-Controller and other computations
    def angle_error(self,  target):
        target_angle = atan2(target.y - self.position.y, target.x - self.position.x)
        return self.normalize(target_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = atan2(sin(angle), cos(angle))
        return angle
        
    # The distance the robot has moved from a defined startig point
    def moved_distance(self,  start):
        return sqrt(pow(self.position.y - start.y, 2) + pow(self.position.x - start.x, 2))

    # The robots distance to the MLine
    def line_distance(self,  goal):
        point_1 = self.start_pos
        point_2 = goal
        point_x = self.position
        numerator = fabs((point_2.y - point_1.y) * point_x.x - (point_2.x - point_1.x) * point_x.y + (point_2.x * point_1.y) - (point_2.y * point_1.x))
        denominator = sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2))
        return numerator / denominator

    # The robots distance to a target
    def euclidean_distance(self,  pos,  target):
        return sqrt(pow((target.x - pos.x), 2) + pow((target.y - pos.y), 2))
        
    #Robot Navigation
    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5
        return msg
    
    def move_right(self):
        msg = Twist()
        msg.linear.x = self.max_vel
        msg.angular.z = -0.5
        return msg

    def move_left(self):
        msg = Twist()
        msg.linear.x = self.max_vel
        msg.angular.z = 0.5
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

########STATE-METHODS and END_PROCEDURE############################################

    # State 0: Robot turns until it looks directly to the target position
    def faceTheGoal(self, target,  tolerance = 0.01):
        if fabs(self.angle_error(target)) > tolerance:
            if (self.angle_error(target) < 0):
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
        if self.front_scanner > self.safety_distance:
                self.twist_msg.angular.z = self.angle_error(goal)
                self.twist_msg.linear.x = self.max_vel
        else:
                self.twist_msg.angular.z = 0
                self.twist_msg.linear.x = 0
                self.wall_hit_point = self.position
                self.point = self.position
                self.change_state(2)
    
    #State 2: Robot follows the wall until it hits the MLine or it doesn't find the MLine again.
    def followWall(self,  goal,  tolerance = 0.1):
        past_pos = self.position
        #General Leaving  condition
        if self.line_distance(goal) <= tolerance and self.moved_distance(self.wall_hit_point) > tolerance and self.euclidean_distance(self.position,  goal) < self.euclidean_distance(self.wall_hit_point,  goal) :
            self.twist_msg = self.stop()
            self.past_positions = [ ]
            self.change_state(0)
        #Leaving condition infinity loop
        elif next((x for x in self.past_positions[:-1] if self.moved_distance(self.past_positions[0]) > self.robot_size and self.euclidean_distance(x, self.position) < 0.01), -1) != -1:
                        self.twist_msg = self.stop()
                        self.change_state(0)
                        self.past_positions = [ ]
        #Case 1: No obstacle
        elif self.scanner_regions['front'] > self.safety_distance and self.scanner_regions['fleft'] > self.safety_distance and self.scanner_regions['bleft'] > self.safety_distance :
            #Case 1.1: Lost wall
            if self.scanner_regions['front'] == self.vision_radius and self.scanner_regions['fleft'] == self.vision_radius and self.scanner_regions['bleft'] == self.vision_radius:
               self.twist_msg = self.stop()
               self.change_state(0)
               self.points = [ ]
            #Case 1.2: Distance to wall bigger than safety_distance
            else:
                self.past_positions.append(past_pos)
                self.twist_msg = self.move_left()
        #Case 2: Obstacle everywhere
        elif self.scanner_regions['front'] <= self.safety_distance and self.scanner_regions['fleft'] <= self.safety_distance and self.scanner_regions['bleft'] <= self.safety_distance and self.scanner_regions['back'] <= self.safety_distance and self.scanner_regions['bright'] <= self.safety_distance and self.scanner_regions['fright'] <= self.safety_distance:
            self.twist_msg = self.stop()
        #Case 3: Obstacle in front of robot
        elif self.scanner_regions['front'] <= self.safety_distance:
            self.twist_msg = self.turn_right()
        #Case 4: Obstacle on the frontleft side
        elif self.scanner_regions['front'] > self.safety_distance and self.scanner_regions['fleft'] <= self.safety_distance and self.scanner_regions['bleft'] > self.safety_distance:
            self.past_positions.append(past_pos)
            self.twist_msg = self.move_right()
        #Case 5: Obstacle on the backleft side
        elif self.scanner_regions['front'] > self.safety_distance and self.scanner_regions['fleft'] > self.safety_distance and self.scanner_regions['bleft'] <= self.safety_distance:
            self.past_positions.append(past_pos)
            self.twist_msg = self.move_left()
        #Case 6: Obstacle on the fronteft and backleft side
        elif self.scanner_regions['front'] > self.safety_distance and self.scanner_regions['fleft'] <= self.safety_distance and self.scanner_regions['bleft'] <= self.safety_distance:
            self.past_positions.append(past_pos)
            #Case 6.1: There is an edge upcoming
            if self.edge_scanner > self.safety_distance:
               self.twist_msg = self.move_left()
            #Case 6.2: No edge
            else:
                self.twist_msg = self.move_forward()
        #Case 7: Unknown Case
        else:
            self.twist_msg = self.stop()
            self.change_state(0)
            
    def end_procedure(self,  tolerance = 0.2):
        if self.end_scenario == 'despawn':
            print('DESPAWN-MODE')
            # is handled by spawning controller
            pass
        elif self.end_scenario == 'stay':
              print('STAY-MODE')
              while True:
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
        elif self.end_scenario == 'idle':
            print('IDLE-MODE')
            while True:
                if self.frontScanner > self.d:
                    self.velocity_publisher.publish(self.move_forward())
                else:
                    self.velocity_publisher.publish(self.turn_right())
                self.rate.sleep()
        elif self.end_scenario == 'start':
            print('START-MODE')
            self.change_state(0)
            while self.euclidean_distance(self.position,  self.init_pos) > self.wp_threshold:
                if self.state == 0:
                    self.faceTheGoal(self.init_pos)
                elif self.state == 1:
                    self.followMLine(self.init_pos)
                elif self.state == 2:
                    self.followWall(self.goal_pos)
                  
                self.velocity_publisher.publish(self.twist_msg)
                self.rate.sleep()
                
            while True:
                self.velocity_publisher.publish(self.stop())
                self.rate.sleep()
                    
########MAIN-METHOD##############################################

    def bug2(self):
        new_goal_pos = Point()
        
        #Main_loop
        while not rospy.is_shutdown():
            
            # Robot finished benchmark
            if self.finished == Bool(True):
                self.velocity_publisher.publish(self.stop())
                self.end_procedure()
                
            # Robot is waiting for a new goal
            elif self.finished == Bool(False) and new_goal_pos == self.goal_pos:
                self.velocity_publisher.publish(self.stop())
                print('Waiting for a new goal')
                
            # Robot has received a new goal and will start with driving towards it
            elif self.finished == Bool(False) and new_goal_pos != self.goal_pos:
                
                if self.init_pos == Point():
                    self.init_pos = self.position
                new_goal_pos = self.goal_pos
                self.start_pos =  self.position
                self.change_state(0)
                print('Goal received')
                
                #Change between the different states
                while self.euclidean_distance(self.position,  self.goal_pos) > self.wp_threshold:
                    if self.state == 0:
                        self.faceTheGoal(self.goal_pos)
                    elif self.state == 1:
                        self.followMLine(self.goal_pos)
                    elif self.state == 2:
                        self.followWall(self.goal_pos)
                        
                    self.velocity_publisher.publish(self.twist_msg)
                    self.rate.sleep()

                print('Found the goal!')
                self.velocity_publisher.publish(self.stop())

            self.rate.sleep()

if __name__ == "__main__" :
    try:
        x = Bug2()
        x.bug2()
    except rospy.ROSInterruptException:
        traceback.print_exc()
