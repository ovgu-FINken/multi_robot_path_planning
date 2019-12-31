#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019/2020
@brief:     This file contains all node and topic names for the benchmark.
@todo:
------------------------------------------------------------- """


from enum import Enum


BENCHMARK_NAMESPACE = "benchmark/"


class NodeNames(Enum):
    """ Enum of all benchmark node names.
    """
    EVALUATION_CONTROLLER = BENCHMARK_NAMESPACE + "evaluation_controller"
    WAYPOINT_CONTROLLER = BENCHMARK_NAMESPACE + "waypoint_controller"
    MOVEMENT_CONTROLLER = BENCHMARK_NAMESPACE + "movement_controller"
    SPAWNING_CONTROLLER = BENCHMARK_NAMESPACE + "spawning_controller"
    SETTINGS_PUBLISHER = BENCHMARK_NAMESPACE + "settings_publisher"
    WORLD_CREATOR = BENCHMARK_NAMESPACE + "world_creator"


class TopicNames(Enum):
    """ Enum of all benchmark topic names.
    """
    FINISHED = BENCHMARK_NAMESPACE + "finished"
    WAYPOINT = BENCHMARK_NAMESPACE + "waypoint"
    START_POSITION = BENCHMARK_NAMESPACE + "start_pos"
