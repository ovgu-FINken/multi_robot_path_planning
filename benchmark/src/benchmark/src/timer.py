#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     Timer for measuring the durations.
@todo:
------------------------------------------------------------- """


import rospy


class Timer:
    """ Timer.
    """

    def __init__(self, number_of_robots):
        """ Init. method.
        :param number_of_robots:
        """
        self._number_of_robots = number_of_robots
        self._timer = {}
        self._init_timer()

    def _init_timer(self):
        """ Init. timers.
        :return timer
        """
        for robot_id in range(self._number_of_robots):
            self._timer[robot_id] = 0.0
        return self._timer

    def start_timer(self, robot_id):
        """ Starts / Restarts the timer for a specific robot.
        :param robot_id
        """
        self._timer[robot_id] = self._get_ros_time()

    def get_time(self, robot_id):
        """ Returns the timer for a specific robot.
        :param robot_id:
        """
        if robot_id not in self._timer:
            return 0.0
        time = self._get_ros_time() - self._timer[robot_id]
        return time

    @staticmethod
    def _get_ros_time():
        """ Wrapper for rospy get time.
        :return time in seconds
        """
        time = rospy.get_time()
        return time

    def is_running(self, robot_id):
        """ Checks whether the timer of a robot is running or not.
        :param robot_id:
        :return True:   is running
                False:  is not running
        """
        if robot_id not in self._timer:
            return False
        if self._timer[robot_id] == 0.0 or self._timer[robot_id] is None:
            return False
        return True
