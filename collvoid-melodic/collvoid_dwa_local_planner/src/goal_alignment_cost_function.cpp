/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2014  Samir Benmendil <samir.benmendil@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "collvoid_dwa_local_planner/goal_alignment_cost_function.h"
//#include <tf/tf.h>
#include <tf2/utils.h>

#include <math.h>
#include <base_local_planner/goal_functions.h>

#include <collvoid_dwa_local_planner/goal_alignment_cost_function.h>

namespace collvoid_dwa_local_planner
{
double GoalAlignmentCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
{
    if (traj.getPointsSize() < 1)
        return 0;

    /*
    // reading yaw from quaternion in message
    tf2::Quaternion q;
    tf2::convert(goalPose_.pose.orientation, q);
    tf2::Matrix3x3 matrix(q);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    //double goalHeading = yaw;
    */
    double yaw = tf2::getYaw(goalPose_.pose.orientation);


    double x, y, th;
    traj.getEndpoint(x, y, th);
    double delta_th = fabs(angles::shortest_angular_distance(th, yaw));
    return delta_th / M_PI;
}
} // namespace collvoid_dwa_local_planner
