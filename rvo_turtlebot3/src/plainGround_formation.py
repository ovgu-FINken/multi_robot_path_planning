#!/usr/bin/env python

import multiprocessing
import rospy
from geometry_msgs.msg import Twist
import math
import numpy as np
import rospkg
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from vo_based_algo_version2 import Turtlebot3_burger
from rvo_turtlebot3.msg import Information
#from std_msgs.msg import Float32


class circle_formation():

    def set_robots(self,sp_x,sp_y,starting_heading):

        self.sp_x = sp_x
        self.sp_y = sp_y
        self.starting_heading = starting_heading

        for i in range(self.user_input):

            self.state_msg = ModelState()
            self.state_msg.model_name =  self.agent_names[i]
            self.state_msg.pose.position.x = self.sp_x[i]
            self.state_msg.pose.position.y = self.sp_y[i]

            (self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w) = quaternion_from_euler(0.0, 0.0, self.starting_heading[i])

            self.state_msg.pose.orientation.x = self.quaternion_x
            self.state_msg.pose.orientation.y = self.quaternion_y
            self.state_msg.pose.orientation.z = self.quaternion_z
            self.state_msg.pose.orientation.w = self.quaternion_w

            print ('Starting_point for agent ', self.agent_names[i])
            print (self.sp_x[i], self.sp_y[i], self.starting_heading[i])

            rospy.wait_for_service('/gazebo/set_model_state')

            try:
                self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                self.resp = self.set_state( self.state_msg )

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

    def circle_calculations(self):

        self.user_input = int(input("Type no. of agents : "))
        ###################################################### ##################### ############ ######## #### ## #
        #self.user_input = 8
        #Initializing lists

        self.agent_names, self.class_obj = [None] * (self.user_input),[None] * (self.user_input)
        self.sp_x,self.sp_y,self.goal_x,self.goal_y, self.starting_heading = [None] * (self.user_input),[None] * (self.user_input),[None] * (self.user_input),[None] * (self.user_input),[None] * (self.user_input)

        #Calculations for starting points and goal points and initial headings for the given number of robots by the user

        self.center_x = 5.0
        self.center_y = 5.0

        self.radius = 2.5

        for i in range(self.user_input):

            self.agent_names[i] = "tb3_" + str(i)
            self.class_obj[i] = 'x' + str(i)

            self.sp_x[i] = self.center_x + np.cos(2*np.pi/self.user_input*i)*self.radius
            self.sp_y[i] = self.center_y + np.sin(2*np.pi/self.user_input*i)*self.radius
            self.goal_x[i] = self.center_x - np.cos(2*np.pi/self.user_input*i)*self.radius
            self.goal_y[i] = self.center_y - np.sin(2*np.pi/self.user_input*i)*self.radius

            #print (sp_x[i], sp_y[i],goal_x[i], goal_y[i])

            self.x_component = self.center_x - self.sp_x[i]
            self.y_component = self.center_y - self.sp_y[i]

            self.starting_heading[i] = round (math.atan2(self.y_component, self.x_component),2)

            #print(agent_names)

        print('Calculated the positions for robots')
        self.set_robots(self.sp_x, self.sp_y, self.starting_heading)
        self.publish_goals(self.agent_names, self.class_obj,self.goal_x, self.goal_y)

    def publish_goals(self, agent_name,class_obj,goal_x, goal_y):

        #self.goal_x_y = [goal_x, goal_y]
        #self.goal_xy_array = np.array(self.goal_x_y)
        print('Now starting the circleformation')

        rospy.init_node('circleformationgoals')

        for _ in range(2):

            self.pub = rospy.Publisher('/goals', Information, queue_size=10)

            self.r = rospy.Rate(10)

            while not rospy.is_shutdown():
                self.msg = Information()
                #self.msg.data = self.goal_xy_array
                self.msg.x = goal_x
                self.msg.y = goal_y
                self.msg.agent_name = agent_name
                self.msg.class_obj = class_obj
                self.pub.publish(self.msg)
                print('Publishing goals')
                print(goal_x)
                print(goal_y)
                self.r.sleep() 

            rospy.loginfo('Goal Node stopped')
            #self.multi_agents(self.agent_names[i], self.class_obj[i], self.goal_x[i], self.goal_y[i])

        rospy.spin()

if __name__ == '__main__':

    class_instance = circle_formation()
    class_instance.circle_calculations()