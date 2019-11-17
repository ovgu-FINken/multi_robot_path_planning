#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Responsible for handling way points.
@todo:
------------------------------------------------------------- """


from enum import Enum


class World(Enum):
    """ Enum of supported world maps.
    """
    EMPTY = "empty_world"
    TURTLEBOT3 = "turtlebot3_world"


class WayPointManager:
    """ Way point manager.
    """

    def __init__(self, world=World.EMPTY):
        """ Init. method.
        :param world
        """
        self._world = world
        self._way_points = []

    def run(self):
        """ Starts the way point generating process.
        :return: way point coordinates
        """
        if self._world == World.EMPTY \
                or self._world == World.EMPTY.value:
            self._set_way_points_empty_world()
        elif self._world == World.TURTLEBOT3 \
                or self._world == World.TURTLEBOT3.value:
            self._set_way_points_tb3_world()
        return self._way_points

    def _set_way_points_empty_world(self):
        """ Sets the way points for the empty world.
        """

    def _set_way_points_tb3_world(self):
        """ Sets the way points for the tb3 world.
        """

