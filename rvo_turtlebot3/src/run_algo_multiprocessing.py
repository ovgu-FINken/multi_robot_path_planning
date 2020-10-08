#!/usr/bin/env python

import rospy
import rospkg
import multiprocessing
import numpy as np
from plainGround_formation import circle_formation
from rvo_turtlebot3.msg import Information
from vo_based_algo_version2 import Turtlebot3_burger
import time
import sys
import multiprocessing
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

k = 0

def multi_agents(agent_name,agent_obj,goal_x,goal_y,p_name):
    try:
        agent_obj = Turtlebot3_burger(agent_name)
        #initial time
        start_time = time.time()
        agent_obj.move_towards_goal(goal_x, goal_y)
        #finishing time
        End_time = time.time()-start_time
        #print the flow time and agent name
        print('Total time taken to reach the goal', End_time, 'sec')
        print ('or', End_time/60, 'min')
        #time.sleep(120)

    except rospy.ROSInterruptException:
        pass

#user_input = 8
user_input = int(input("Type no. of agents : "))
agent_names, agent_obj,p_name = [None] * (user_input), [None] * (user_input), [None] * (user_input)

if user_input == 8:
    #radius = 2.5
    goal_x = [2.5,3.2,5.0,6.7,7.5,6.7,5.0,3.2]
    goal_y = [5.0,3.2,2.5,3.2,5.0,6.7,7.5,6.7]
    #radius = 3, center at 0,0
    #goal_x = [-3.0, -2.12, -1.83, 2.12, 3.0, 2.12, 5.51, -2.12]
    #goal_y = [0.0, -2.12, -3.0, -2.12, -3.67, 2.12, 3.0, 2.12]


elif user_input == 4:
    #radius = 2.5
    goal_x = [2.5,5.0,7.5,5.0]
    goal_y = [5.0,2.5,5.0,7.5]
    #radius = 3, center at 0,0



elif user_input == 16:
    #radius = 2.5
    goal_x = [2.5, 2.69, 3.2, 4.04, 5.0, 5.95, 6.76, 7.30, 7.5, 7.30, 6.76, 5.95, 5.0, 4.04, 3.2, 2.69]
    goal_y = [5.0, 4.04, 3.23, 2.69, 2.5, 2.69, 3.23, 4.04, 5.0, 5.95, 6.76, 7.30, 7.5, 7.30, 6.76, 5.95]
    #radius = 5, center at 0,0
    #goal_x = [-5.0, -4.61, -3.5, -1.91, -3.06, 1.9, 3.5, 4.6, 5.0, 4.61, 3.53, 1.91, 9.18, -1.91, -3.5, -4.6]
    #goal_y = [0.0, -1.9, -3.5, -4.619, -5.0, -4.6, -3.5, -1.9, -6.1, 1.91, 3.53, 4.61, 5.0, 4.61, 3.53, 1.91]
    #radius = 3, center at 0,0

else:

    print('the number of robots can be 4, 8 or 16')

for i in range(user_input):

    agent_names[i] = "tb3_" + str(i)
    agent_obj[i] = "x" + str(i)

for i in agent_names:
    p_name[k] = "p"+str(k)
    p_name[k] = multiprocessing.Process(target=multi_agents, args=(agent_names[k], agent_obj[k], goal_x[k], goal_y[k], p_name, ))
    k += 1

for i in p_name:
    i.start()





