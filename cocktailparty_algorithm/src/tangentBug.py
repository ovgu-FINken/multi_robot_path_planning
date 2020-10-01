#!/usr/bin/env python
import rospy
import traceback
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, radians, fabs, pi, cos, sin, ceil,  degrees

class TangentBug:

    state_dict = {
        0: 'Motion to goal',
        1: 'Transition', 
        2: 'Following obstacle', 
        3: 'Goal reached'
    }

    def __init__(self):
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
        self.end_scenario = rospy.get_param("/end_procedure")
        self.robot_name = rospy.get_param('~robot_name')
        self.wp_threshold = rospy.get_param("/wp_threshold")
        self.vision_radius = rospy.get_param("~vision_radius")
        self.robot_size = rospy.get_param("~robot_size")
        self.max_vel = rospy.get_param("~max_vel")
        rate = rospy.get_param("~rate")
        self.collision_scanner_size = rospy.get_param("~collision_scanner_size")
        self.safety_distance = rospy.get_param("~safety_distance")
        self.wall_distance = rospy.get_param("~wall_distance")
        self.rate = rospy.Rate(rate)
        self.position = Point()
        self.past_pos = Point()
        self.goal_pos = Point()
        self.start_pos = Point()
        self.init_pos = Point()
        self.waypoint = Point()
        self.twist_msg = Twist()
        self.finished = Bool()
        self.scanner = []
        self.past_positions = []
        self.regions = []
        self.edge_scanners = []
        self.region = ''
        self.front_scanner = 0
        self.collision_scanner = 0
        self.orientation = 0.0
        self.state = 0
        self.d_follow = -1
        self.d_reach = -1
        self.direction= 0
        self.heuristic_distance = -1
        
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
        self.orientation = theta # [-pi,pi]

    def callback_laser(self, data):
        self.scanner = data.ranges[:]
        self.region ={
            'front' : min(min(data.ranges[0:30]),min(data.ranges[330:360]),self.vision_radius),
            'fleft' : min(min(data.ranges[30:90]),self.vision_radius),
            'bleft' : min(min(data.ranges[90:150]),self.vision_radius),
            'back' : min(min(data.ranges[150:210]),self.vision_radius),
            'bright' : min(min(data.ranges[210:270]),self.vision_radius), 
            'fright' : min(min(data.ranges[270:330]),self.vision_radius)
            }
        self.collision_scanner = min(min(data.ranges[0:self.collision_scanner_size/2]),min(data.ranges[360 - self.collision_scanner_size/2 :360]),self.vision_radius)
        self.edge_scanners = [min(min(self.scanner[30:60]),self.vision_radius),  min(min(self.scanner[300:330]),self.vision_radius) ]

####################HELPER-METHODS##############################################
    
    # Calculates scanner size
    def calc_robot_gap(self,  radius):
        scanner_size =  int(ceil(((self.robot_size +  self.wall_distance + 0.2 ) * 360.0) /( 2.0 * pi * radius))) 
        if self.is_even(scanner_size) != True :
            scanner_size += 1
        return scanner_size
        
    # Checks if a number is even 
    def is_even(self, number):
        mod = number % 2
        if mod > 0:
            return False
        else:
            return True

    # Angle error for the P-Controller
    def angle_error(self, target):
        goal_angle = atan2(target.y - self.position.y, target.x - self.position.x)
        return self.normalize(goal_angle - self.orientation)

    # Normalize given angle
    def normalize(self, angle):
        if fabs(angle) > pi:
            angle = angle - (2 * pi * angle) / (fabs(angle))
        return angle
        
    #Normalize given angle
    def normalize_deg(self,  angle):
        if angle >= 360:
            angle -= 360
        elif angle < 0:
            angle += 360
        return angle

    # The robots distance to a target
    def euclidean_distance(self, pos, target):
        return sqrt(pow((target.x - pos.x), 2) + pow((target.y - pos.y), 2))
    
    # Calculates next waypoint (goTangent)
    def calc_next_wp(self,  goal_pos):
        points = self.get_discontinuity_points()
        if len(points) == 0:
            return None
        heuristic = self.euclidean_distance(self.position, points[0]) + self.euclidean_distance(points[0], goal_pos)
        next_point = points[0]
        for i in range(1,len(points)):
            temp = self.euclidean_distance(self.position, points[i]) + self.euclidean_distance(points[i], goal_pos)
            if temp < heuristic:
                heuristic = temp
                next_point = points[i]
        if self.heuristic_distance == -1:
            self.heuristic_distance = heuristic
        else:
            if heuristic > self.heuristic_distance:
                return None
            else:
                self.heuristic_distance = heuristic
                
        return next_point
    
    #Calculate discontinuty points (goTangent)
    def get_discontinuity_points(self):
        points = []
        scanner = []
        scanner.extend(self.scanner[:])
        scanner.extend(self.scanner[:])
        scanner.extend(self.scanner[:])
        flag = -99
        
        i = 360 
        if scanner[i] >= (self.vision_radius):
            flag = 0 #no Obstacle
        else:
            flag = 1 #Obstacle
        i += 1
        
        while i <= (720):
            if scanner[i] >= (self.vision_radius):
                temp_flag = 0
                #negative flag:  obstacle -> no obstacle
                if temp_flag != flag:
                    flag = temp_flag
                    radius = scanner[i-1]
                    if radius > self.vision_radius:
                        radius = self.vision_radius
                    gap_size = self.calc_robot_gap(radius)/2
                    if min(scanner[i:i+gap_size]) >= self.vision_radius:
                        alpha_deg =self.normalize_deg(i - 360  + gap_size )
                        alpha_rad = self.normalize(radians(alpha_deg))
                        x = self.position.x + (scanner[i-1] ) * cos(self.orientation + alpha_rad ) - 0.1/sqrt(2)
                        y = self.position.y + (scanner[i-1] ) * sin(self.orientation + alpha_rad ) + 0.1/sqrt(2)
                        points.append(Point(x, y, 0.0))
            else:
                temp_flag = 1
                #positive flag: no obstacle -> obstacle
                if temp_flag != flag:
                    flag = temp_flag
                    radius = scanner[i]
                    if radius > self.vision_radius:
                        radius = self.vision_radius
                    gap_size = self.calc_robot_gap(radius)
                    if min(scanner[i-gap_size:i]) >= self.vision_radius:
                        alpha_deg = self.normalize_deg(i - 360 - gap_size) 
                        alpha_rad = self.normalize(radians(alpha_deg) )
                        x = self.position.x + (scanner[i]) * cos(self.orientation  + alpha_rad ) + 0.1/sqrt(2)
                        y = self.position.y +(scanner[i]) * sin(self.orientation + alpha_rad ) - 0.1/sqrt(2)
                        points.append(Point(x, y, 0.0))
            i += 1
        return points
    
    #Calculated d follow (wall following)
    def get_d_follow(self,  goal_pos):
        d_follow = self.d_follow
        if self.direction == 1:
            scanner = self.scanner[0:180]
        else:
            scanner = self.scanner[180:360]
        follow_point = Point()
        for i in range(len(scanner)):
            if scanner[i] <= self.vision_radius: 
                x = self.position.x + (scanner[i]) * cos(self.orientation + self.normalize(radians(i)))
                y = self.position.y +(scanner[i] ) * sin(self.orientation + self.normalize(radians(i)))
                if follow_point == Point():
                    follow_point = Point(x, y, 0.0)
                else:
                    point_temp = Point(x, y, 0.0)
                    if self.euclidean_distance(point_temp,  self.goal_pos) < self.euclidean_distance(follow_point,  self.goal_pos) and self.euclidean_distance(point_temp,  self.position) < self.euclidean_distance(follow_point,  self.position):
                        follow_point = point_temp
                        
        d_follow = self.euclidean_distance(follow_point, goal_pos) 
        return d_follow
        
    #Robot navigation
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
    
    # Robot turns to a specific point
    def turn_2point(self, point, tolerance = 0.01):
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
    
    #Robot moves to a specific point
    def move_2point(self, waypoint,  collision_avoidance,  tolerance = 0.1):
        msg = Twist()
        while(self.euclidean_distance(self.position, waypoint) > tolerance ):
            if self.collision_scanner > self.safety_distance or collision_avoidance == False:
                msg.angular.z = self.angle_error(waypoint) 
                msg.linear.x = self.max_vel 
                self.velocity_publisher.publish(msg)
                self.rate.sleep()
            else:
                msg.angular.z = 0
                msg.linear.x = 0
                self.velocity_publisher.publish(msg) 
                return False
        self.velocity_publisher.publish(self.stop())
        return True
    
    #Sets the robots travel direction
    def set_direction(self):
        if self.region['fleft'] < self.region['fright']:
            self.direction = 1
        else:
            self.direction = -1
        
#####################STATE-METHODS AND END_PROCEDURE##############################################
    
    # Change between the different states
    def change_state(self, state):
        if state is not self.state:
            print ('[%s] - State %s: %s' % (self.robot_name, state, self.state_dict[state]))
            self.state = state
   
    # State 0:
    def motion_2goal(self,  goal_pos):
        robot_gap = self.calc_robot_gap(self.euclidean_distance(self.position,  goal_pos))
        scanner = []
        scanner.extend(self.scanner[360-robot_gap : 360])
        scanner.extend(self.scanner[:])
        scanner.extend(self.scanner[0:robot_gap])
        goal_angle = int(ceil(self.normalize_deg(degrees( self.angle_error(goal_pos)))))
        goal_scanner = min(scanner[int(robot_gap/2 + goal_angle) : int(1.5*robot_gap+ goal_angle)])

        if goal_scanner >= self.vision_radius or self.euclidean_distance(self.position,  goal_pos) < goal_scanner :
           self.go_straight(goal_pos) #Go straight
        else:
            self.go_tangent(goal_pos) # Go tangent
            
    #State 0.1:
    def go_straight(self,  goal_pos): 
        print ('GO STRAIGHT') 
        self.turn_2point(goal_pos)
        self.set_direction()
        if self.euclidean_distance(self.position, goal_pos) >= self.vision_radius:
            const = sqrt(pow(self.vision_radius - self.wall_distance, 2) / (pow(goal_pos.x - self.position.x, 2) + pow(goal_pos.y - self.position.y, 2)))
            self.waypoint.x =  self.position.x + const * (goal_pos.x - self.position.x)
            self.waypoint.y =  self.position.y + const * (goal_pos.y - self.position.y)
            self.waypoint.z = 0
            collision_avoidance = True
        elif self.euclidean_distance(self.position,  goal_pos) >= self.safety_distance:
            self.waypoint = goal_pos
            collision_avoidance = True
        elif self.euclidean_distance(self.position,  goal_pos) < self.safety_distance and self.collision_scanner > self.safety_distance:
            self.waypoint = goal_pos
            collision_avoidance = False
        else:
            self.waypoint = goal_pos
            collision_avoidance = True
        
        value = self.move_2point(self.waypoint,  collision_avoidance)
        #Collision avoidance
        if value == False :
           print('Collision!')
           if self.direction == -1:
                self.d_follow = self.get_d_follow
                self.past_pos = self.position
                self.change_state(1)
           else:
                count = 0
                while self.collision_scanner <= self.safety_distance:
                    self.velocity_publisher.publish(self.stop())
                    self.rate.sleep()
                    if count == 250:
                        break
                    count+=1
         
    #State 0.2:
    def go_tangent(self,  goal_pos):
        print('GO TANGENT!')
        next_point = self.calc_next_wp(goal_pos)
        if next_point is not None:
            self.waypoint = next_point
            self.turn_2point(self.waypoint)
            self.set_direction()
            value = self.move_2point(self.waypoint,  True)
            #Collision avoidance
            if value == False:
                print('Collision!')
                if self.direction == -1:
                    self.change_state(1)
                    self.past_pos = self.position
                    self.d_follow = self.get_d_follow(goal_pos)
                else:
                    count = 0
                    while self.collision_scanner <= self.safety_distance:
                        self.velocity_publisher.publish(self.stop())
                        self.rate.sleep()
                        if count == 250:
                            break
                        count+=1
        else:
            self.d_follow = self.get_d_follow(goal_pos)
            self.past_pos = self.position
            self.change_state(1)
            
    # State 1:
    def transition(self,  goal_pos):
        if  self.direction == 1:
            self.regions = ['front',  'fleft',  'bleft',  'back',  'bright',  'fright']
            self.edge_scanner =  0
        else:
            self.regions = ['front',  'fright',  'bright',  'back',  'bleft',  'fleft']
            self.edge_scanner =  1
        self.change_state(2)

    # State 2:
    def follow_wall(self,  goal_pos):
        self.d_reach = self.euclidean_distance(self.position,  goal_pos)
        # General leaving condition
        if self.d_reach  + 0.05 < self.d_follow:
            self.twist_msg = self.stop()
            self.change_state(0)
            self.heuristic_distance = -1
            self.past_positions = [ ]
        # Infinity Cycle
        elif next((x for x in self.past_positions[:-1]  if  self.euclidean_distance(x, self.position) <= 0.01) , -1) != -1:
            self.velocity_publisher.publish(self.stop())
            self.change_state(0)
            self.heuristic_distance = -1
            self.past_positions= []
        #Case1: No obstacle
        elif self.region[self.regions[0]] > self.safety_distance and self.region[self.regions[1]] > self.safety_distance and self.region[self.regions[2]] > self.safety_distance :
           #Case 1.1 Lost wall
            if self.region[self.regions[0]] >= self.vision_radius and self.region[self.regions[1]] >=  self.vision_radius  and self.region[self.regions[2]]  >= self.vision_radius :
                self.twist_msg = self.stop()
                self.change_state(0)
                self.heuristic_distance = -1
                self.past_positions = [ ]
            #Case 1.2: Distance to wall bigger than safety distance
            else:
                self.twist_msg = self.move(self.direction)
        #Case 2: Obstacle everywhere
        elif self.region[self.regions[0]]<= self.safety_distance and self.region[self.regions[1]] <= self.safety_distance and self.region[self.regions[2]]  <= self.safety_distance and self.region[self.regions[3]] <= self.safety_distance and self.region[self.regions[4]] <= self.safety_distance and self.region[self.regions[5]] <= self.safety_distance:
            self.twist_msg = self.stop()
        #Case 3: Wall in front
        elif self.region[self.regions[0]] <= self.safety_distance:
            if self.direction == -1:
                self.twist_msg = self.turn(-1*self.direction)
            else:
                count = 0
                while self.region[self.regions[0]] <= self.safety_distance:
                    self.velocity_publisher.publish(self.stop())
                    if count == 100:
                        self.velocity_publisher.publish(self.turn(-1*self.direction))
                        count-= 1
                    count+=1
                    self.rate.sleep()
        #Case 4: Obstacle on the front left / front right side
        elif self.region[self.regions[0]] > self.safety_distance and self.region[self.regions[1]] <= self.safety_distance and self.region[self.regions[2]]  > self.safety_distance:
            self.twist_msg = self.move(-1*self.direction)
        # Case 5: Obstacle on the back left / back right side
        elif self.region[self.regions[0]] > self.safety_distance and self.region[self.regions[1]] > self.safety_distance and self.region[self.regions[2]]  <= self.safety_distance:
            self.twist_msg = self.move(self.direction)
        # Case 6: Obstacle on the front left and back left / front right and back right side
        elif self.region[self.regions[0]] > self.safety_distance and self.region[self.regions[1]] <= self.safety_distance and self.region[self.regions[2]]  <= self.safety_distance:
            # Case 6.1: There is an edge upcoming
            if self.edge_scanners[self.edge_scanner] >= self.wall_distance:
                self.twist_msg = self.move(self.direction)
            # Case 6.2: No edge
            else:
                self.twist_msg = self.move_forward()
        # Case 7: Unknown case
        else:
            print('Unknown case!')
            self.twist_msg = self.stop()
            self.change_state(0)
            self.heuristic_distance = -1
            self.past_positions = [ ]

        self.velocity_publisher.publish(self.twist_msg)
        
    #State 4: Goal reached
    def goal_reached(self):
        self.heuristic_distance = -1
        self.velocity_publisher.publish(self.stop())
        self.regions = []
    
    # End_Procedure after finishing benchmark
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
                if self.regions['front'] > self.wall_distance:
                    self.velocity_publisher.publish(self.move_forward())
                else:
                    self.velocity_publisher.publish(self.turn_right())
                self.rate.sleep()
        elif self.end_scenario == 'start':
            print('START-MODE')
            self.turn_2point(self.init_pos)
            self.change_state(0)
            while self.euclidean_distance(self.position, self.init_pos) > self.wp_threshold:
                    if self.state == 0:
                        self.motion_2goal(self.init_pos)
                    elif self.state == 1:
                        self.transition(self.init_pos)
                    elif self.state == 2:
                        self.follow_wall(self.init_pos)
                        if self.euclidean_distance(self.past_pos,  self.position) >= 0.1:
                            self.past_positions.append(self.past_pos)
                            self.past_pos = self.position
                    
                    self.rate.sleep()
                    self.d_reach = self.euclidean_distance(self.position, self.init_pos)
                    
            while True:
                self.velocity_publisher.publish(self.stop())
                self.rate.sleep()
                
###########################MAIN-LOOP########################################
    def tangentBug(self):
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
                print('[%s] - Turning to goal' % (self.robot_name))
                self.turn_2point(new_goal_pos)
                self.change_state(0)

                #Robot is driving towards the goal
                while self.euclidean_distance(self.position, new_goal_pos) > self.wp_threshold:
                    if self.state == 0:
                        self.motion_2goal(new_goal_pos)
                    elif self.state == 1:
                        self.transition(new_goal_pos)
                    elif self.state == 2:
                        self.follow_wall(new_goal_pos)
                        if self.euclidean_distance(self.past_pos,  self.position) >= 0.1:
                            self.past_positions.append(self.past_pos)
                            self.past_pos = self.position
                        
                    self.rate.sleep()
                    self.d_reach = self.euclidean_distance(self.position, new_goal_pos)
                    
                #Robot has found the current goal
                self.change_state(3)
                self.goal_reached()

            self.rate.sleep()

##############################MAIN##############################################

if __name__ == "__main__" :
    try:
        x = TangentBug()
        x.tangentBug()
    except rospy.ROSInterruptException:
        traceback.print_exc()
