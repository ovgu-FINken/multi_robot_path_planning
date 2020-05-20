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
    RANDOM = "random"


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
        elif self._formation == Formation.RANDOM.value:
            self._estimate_random()
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
