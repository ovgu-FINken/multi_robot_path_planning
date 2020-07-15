#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Computes the positions and orientation
            for a certain formation.
@todo:
------------------------------------------------------------- """

from enum import Enum
import rospy
import math


DEFAULT_DISTANCE = 0.2


class Formation(Enum):
    """ Enum of supported robot formations.
    """
    DENSE_BLOCK = "dense_block"
    AT_WAY_POINTS = "at_way_point"
    TWO_ROOMS = "two_rooms"


class FormationHandler:
    """ Formation Handler.
    """

    def __init__(self, number_of_robots, center_point,
                 formation=Formation.DENSE_BLOCK,
                 distance=DEFAULT_DISTANCE):
        """ Init. Method.
        :param number_of_robots
        :param formation
        :param center_point
        :param distance: This will only be used in appropriate formations.
                         (DENSE_BLOCK)
        """
        self._center_point = center_point
        self._formation = formation
        self._number_of_robots = number_of_robots
        self._distance = distance
        self._position = []
        self._orientation = []

    def run(self):
        """ Runs the computation process and returns the positions and orientations.
        :return: positions and orientations
        """
        if self._formation == Formation.DENSE_BLOCK \
                or self._formation == Formation.DENSE_BLOCK.value:
            self._estimate_dense_block()
        elif self._formation == Formation.AT_WAY_POINTS \
                or self._formation == Formation.AT_WAY_POINTS.value:
            self._estimate_at_way_points()
        elif self._formation == Formation.TWO_ROOMS.value:
            self._estimate_two_rooms()
        else:
            rospy.logerr(
                "Invalid formation {} specified!".format(self._formation))
        return self._position, self._orientation

    def _estimate_dense_block(self):
        """ Runs the computation process for a dense block formation
        and returns the positions and orientations.
        """
        if self._number_of_robots == 1 or self._number_of_robots == 2:
            rospy.loginfo("Calc. trivial dense block")
            self._handle_trivial_dense_block()
        else:
            rospy.loginfo("Calc. non-trivial dense block")
            if self._number_of_robots==3:
                block_dimension = int(
                    math.ceil(math.sqrt(self._number_of_robots)))
            else:
                block_dimension = int(math.sqrt(self._number_of_robots))
            block_rest = self._number_of_robots - 2 * block_dimension
            num_rows = block_dimension + (0 if block_rest == 0 else 1)
            for row in range(num_rows):
                row_pos = row * self._distance
                num_col = block_dimension if row < block_dimension else block_rest
                for col in range(num_col):
                    col_pos = col * self._distance
                    self._position.append([
                        row_pos + self._center_point[0],
                        col_pos + self._center_point[1], 0.0])
                    self._orientation.append([0.0, 0.0, 0.0])

    def _handle_trivial_dense_block(self):
        """ Sets the postion and the orientation for trivial dense block
        formations, like for one or two robots.
        """
        if self._number_of_robots == 1:
            self._position.append(self._center_point)
            self._orientation.append([0.0, 0.0, 0.0])
        elif self._number_of_robots == 2:
            self._position.append([
                self._center_point[0],
                self._center_point[1],
                0.0
            ])
            self._orientation.append([0.0, 0.0, 0.0])
            self._position.append([
                self._center_point[0] + self._distance,
                self._center_point[1],
                0.0
            ])
            self._orientation.append([0.0, 0.0, 0.0])

    def _estimate_at_way_points(self):
        """ Runs the computation process for a at way points formation
        and returns the positions and orientations.
        """
        rospy.loginfo("There is no such formation specified!")

    def _estimate_two_rooms(self):
        """ Runs the computation process for two_rooms spawning
        and returns the positions and orientations.
        """
        if self._number_of_robots > 16:
            rospy.loginfo("Too many robots!!")
        else:
            rospy.loginfo("Calc. two_rooms formation")

            distance = self._distance / 2

            # cp0 = [1.0 , 2.0, 0.0]
            # cp1 = [1.0, -1.0, 0.0]
            # cp2 = [-1.0, 2.0, 0.0]
            # cp3 = [-1.0, -1.0, 0.0]

            ### alternatively starting closer to the center of each room
            cp0 = [0.5 , 2.0, 0.0]
            cp1 = [0.5, -1.0, 0.0]
            cp2 = [-0.5, 2.0, 0.0]
            cp3 = [-0.5, -1.0, 0.0]

            cp = [cp0,cp1,cp2,cp3]
            points = [0,0,0,0]

            for i in range(4):
                points[i]= [[cp[i][0]+distance, cp[i][1]+distance, 0.0],
                            [cp[i][0]-distance, cp[i][1]-distance, 0.0],
                            [cp[i][0]+distance, cp[i][1]-distance, 0.0],
                            [cp[i][0]-distance, cp[i][1]+distance, 0.0]]

            for i in range(self._number_of_robots):
                if i in set([0,4,8,12]):
                    item = points[0].pop(0)
                    self._position.append(item)
                    self._orientation.append([0.0, 0.0, 0.0])
                elif i in set([1,5,9,13]):
                    item = points[1].pop(0)
                    self._position.append(item)
                    self._orientation.append([0.0, 0.0, 0.0])
                elif i in set([2,6,10,14]):
                    item = points[2].pop(0)
                    self._position.append(item)
                    self._orientation.append([0.0, 0.0, 0.0])
                elif i in set([3,7,11,15]):
                    item = points[3].pop(0)
                    self._position.append(item)
                    self._orientation.append([0.0, 0.0, 0.0])
