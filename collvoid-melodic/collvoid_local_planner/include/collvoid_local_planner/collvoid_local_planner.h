/*
 * Copyright (c) 2012, Daniel Claes, Maastricht University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maastricht University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef COLLVOID_LOCAL_PLANNER_H_
#define COLLVOID_LOCAL_PLANNER_H_

#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometr_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>

#include "collvoid_local_planner/ROSAgent.h"
#include "collvoid_local_planner/CollvoidConfig.h"
//#include <base_local_planner/trajectory_planner_ros.h>

using namespace collvoid;

namespace collvoid_local_planner
{
/* typedef boost::shared_ptr<ROSAgent> ROSAgentPtr; */

class CollvoidLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
    CollvoidLocalPlanner();
    //CollvoidLocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
       * @brief  Virtual destructor for the interface
       */
    ~CollvoidLocalPlanner();
    /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
    virtual bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) = 0;

    /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
    virtual bool isGoalReached() = 0;

    /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) = 0;

    /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
    virtual void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) = 0;
    inline bool isInitialized() const { return initialized_; } //COLLVOID

private:
    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);
    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

    void findBestWaypoint(geometry_msgs::PoseStamped &target_pose, const tf::geometry_msgs::PoseStamped &global_pose);

    bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    //Dyn reconfigure
    void reconfigureCB(collvoid_local_planner::CollvoidConfig &config, uint32_t level);
    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig> *dsrv_;
    collvoid_local_planner::CollvoidConfig default_config_;

    //Datatypes:
    costmap_2d::Costmap2DROS *costmap_ros_;
    tf::TransformListener *tf_;
    base_local_planner::LocalPlannerUtil planner_util_;
    base_local_planner::OdometryHelperRos odom_helper_;
    std::string odom_topic_;

    base_local_planner::LatchedStopRotateController latchedStopRotateController_;
    geometry_msgs::PoseStamped current_pose_;

    bool initialized_, skip_next_, setup_;

    unsigned int current_waypoint_;
    //ROS agent
    ROSAgentPtr me_;

    std::vector<geometry_msgs::PoseStamped> transformed_plan_;
    ros::Publisher g_plan_pub_, l_plan_pub_;

    ros::ServiceServer clear_costmaps_srv_;

}; //end class

//  Vector2 rotateVectorByAngle(double x, double y, double ang);

}; // namespace collvoid_local_planner

#endif //end guard catch
