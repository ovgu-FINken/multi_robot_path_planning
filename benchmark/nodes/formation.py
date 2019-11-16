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


class Formation(Enum):
    """ Enum of supported robot formations.
    """
    DENSE_BLOCK = 0
    AT_WAY_POINTS = 1


class FormationHandler:
    """ Formation Handler.
    """

    def __init__(self, number_of_robots, formation=Formation.DENSE_BLOCK):
        """ Init. Method.
        :param number_of_robots
        :param formation
        """
        self.formation = formation
        self.number_of_robots = number_of_robots

    def run(self):
        """ Runs the computation process and returns the positions and orientations.
        :return: positions and orientations
        """
        if self.formation == Formation.DENSE_BLOCK \
                or self.formation == Formation.DENSE_BLOCK.value:
            return self._estimate_dense_block()
        elif self.formation == Formation.AT_WAY_POINTS \
                or self.formation == Formation.AT_WAY_POINTS.value:
            return self._estimate_at_way_points()
        else:
            rospy.logerr("Invalid formation {} specified!".format(self.formation))
            return None
        
    def _estimate_dense_block(self):
        """ Runs the computation process for a dense block formation
        and returns the positions and orientations.
        :return: positions and orientations
        """

    def _estimate_at_way_points(self):
        """ Runs the computation process for a at way points formation
        and returns the positions and orientations.
        :return: positions and orientations
        """