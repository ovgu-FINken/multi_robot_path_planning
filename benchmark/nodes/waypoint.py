#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Responsible for handling way points.
@todo:
------------------------------------------------------------- """


from enum import Enum


class WayPointMap(Enum):
    """ Enum of supported way point maps.
    """
    EMPTY_WORLD = [
        [0.0, 0.0, 0.0]
    ]
    EDGE_TB3_WORLD = [
        [1.8, 0.0, 0.0],
        [-1.8, 0.0, 0.0],
        [0.0, 1.8, 0.0],
        [0.0, -1.8, 0.0]
    ]


class WayPointManager:
    """ Way point manager.
    """

    def __init__(self, way_points=WayPointMap.EMPTY_WORLD):
        """ Init. method.
        :param way_points
        """
        self._way_points = way_points

    def run(self):
        """ Starts the way point generating process.
        """
        if self._way_points == WayPointMap.EMPTY_WORLD \
                or self._way_points == WayPointMap.EMPTY_WORLD.value:
            self._set_way_points_empty_world()
        elif self._way_points == WayPointMap.EDGE_TB3_WORLD \
                or self._way_points == WayPointMap.EDGE_TB3_WORLD.value:
            self._set_way_points_tb3_world()

    def _set_way_points_empty_world(self):
        """ Sets the way points for the empty world.
        """

    def _set_way_points_tb3_world(self):
        """ Sets the way points for the tb3 world.
        """


