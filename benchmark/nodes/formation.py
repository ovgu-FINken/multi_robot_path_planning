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


DEFAULT_DISTANCE = 5


class Formation(Enum):
    """ Enum of supported robot formations.
    """
    DENSE_BLOCK = 0
    AT_WAY_POINTS = 1


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
        else:
            rospy.logerr("Invalid formation {} specified!".format(self._formation))
        return self._position, self._orientation

    def _estimate_dense_block(self):
        """ Runs the computation process for a dense block formation
        and returns the positions and orientations.
        """
        row_pos = 0
        block_dimension = int(math.sqrt(self._number_of_robots))
        block_rest = self._number_of_robots - 2 * block_dimension
        for row in range(block_dimension):
            row_pos = row * self._distance
            for col in range(block_dimension):
                col_pos = col * self._distance
                self._position.append([row_pos, col_pos, 0])
                self._orientation.append([0, 0, 0])
        for num in range(block_rest):
            row_pos += row_pos + self._distance
            for col in range(block_dimension):
                col_pos = col * self._distance
                self._position.append([row_pos, col_pos, 0])
                self._orientation.append([0, 0, 0])

    def _estimate_at_way_points(self):
        """ Runs the computation process for a at way points formation
        and returns the positions and orientations.
        """