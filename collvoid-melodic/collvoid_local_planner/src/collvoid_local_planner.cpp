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

#include <pluginlib/class_list_macros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <std_srvs/Empty.h>
#include <costmap_2d/obstacle_layer.h>
#include <tf2/utils.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "collvoid_local_planner/collvoid_local_planner.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(collvoid_local_planner::CollvoidLocalPlanner, nav_core::BaseLocalPlanner)

template <typename T>
void getParam(const ros::NodeHandle nh, const string &name, T *place)
{
    bool found = nh.getParam(name, *place);
    ROS_ASSERT_MSG(found, "Could not find parameter %s", name.c_str());
    ROS_DEBUG_STREAM_NAMED("init", std::string("Initialized ") << name << " to " << *place);
}

template <class T>
T getParamDef(const ros::NodeHandle nh, const string &name, const T &default_val)
{
    T val;
    nh.param(name, val, default_val);
    ROS_DEBUG_STREAM_NAMED("init", std::string("Initialized ") << name << " to " << val << "(default was " << default_val << ")");
    return val;
}

namespace collvoid_local_planner
{

    CollvoidLocalPlanner::CollvoidLocalPlanner() : odom_helper_("odom"),
                                                   initialized_(false)
    {
    }

    CollvoidLocalPlanner::~CollvoidLocalPlanner()
    {
        delete dsrv_;
    }

    void CollvoidLocalPlanner::initialize(
        std::string name,
        tf2_ros::Buffer *tf,
        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized())
        {

            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);
            current_waypoint_ = 0;
            ros::NodeHandle private_nh("~/" + name);

            // make sure to update the costmap we'll use for this cycle
            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
            //ros::NodeHandle nh;
            me_ = ROSAgentPtr(new ROSAgent);
            me_->init(private_nh, tf, &planner_util_, costmap_ros_);

            skip_next_ = false;
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

            if (private_nh.getParam("odom_topic", odom_topic_))
            {
                odom_helper_.setOdomTopic(odom_topic_);
            }

            setup_ = false;

            //intialize the collision planner
            //collision_planner_.initialize("collision_planner", tf_, costmap_ros_);
            //
            dsrv_ = new dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>(private_nh);

            dynamic_reconfigure::Server<collvoid_local_planner::CollvoidConfig>::CallbackType cb = boost::bind(
                &CollvoidLocalPlanner::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            ros::NodeHandle nh("~/");
            clear_costmaps_srv_ = nh.advertiseService("clear_local_costmap", &CollvoidLocalPlanner::clearCostmapsService, this);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        //end if init
    } // end init

    bool CollvoidLocalPlanner::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        //clear the costmaps
        std::vector<boost::shared_ptr<costmap_2d::Layer>> *plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
        for (std::vector<boost::shared_ptr<costmap_2d::Layer>>::iterator layer = plugins->begin(); layer != plugins->end(); ++layer)
        {
            boost::shared_ptr<costmap_2d::ObstacleLayer> obstacle_layer = boost::dynamic_pointer_cast<costmap_2d::ObstacleLayer>(*layer);
            if (!obstacle_layer)
            {
                // ROS_INFO("NO Obstacle layer\n");
                continue;
            }
            else
            {
                //ROS_INFO("REMOVING Obstacle layer\n");
                (*layer)->reset();
            }
        }
        costmap_ros_->getLayeredCostmap()->updateMap(0, 0, 0);
        costmap_ros_->updateMap();
        //ros::Duration(0.5).sleep()/home/nele/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_local_planner/src/ROSAgent.cpp: In member function ‘bool collvoid::ROSAgent::getTwistServiceCB(collvoid_local_planner::GetCollvoidTwist::Request&, collvoid_local_planner::GetCollvoidTwist::Response&)’:
        return true;
    }

    void CollvoidLocalPlanner::reconfigureCB(collvoid_local_planner::CollvoidConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if (setup_ && config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }

        if (!setup_)
        {
            default_config_ = config;
            setup_ = true;
        }

        // update generic local planner params
        base_local_planner::LocalPlannerLimits limits;
        limits.max_vel_trans = config.max_vel_trans;
        limits.min_vel_trans = config.min_vel_trans;
        limits.max_vel_x = config.max_vel_x;
        limits.min_vel_x = config.min_vel_x;
        limits.max_vel_y = config.max_vel_y;
        limits.min_vel_y = config.min_vel_y;
        limits.max_vel_theta = config.max_vel_theta;
        limits.min_vel_theta = config.min_vel_theta;
        limits.acc_lim_x = config.acc_lim_x;
        limits.acc_lim_y = config.acc_lim_y;
        limits.acc_lim_theta = config.acc_lim_theta;
        limits.acc_lim_trans = config.acc_lim_trans;
        limits.xy_goal_tolerance = config.xy_goal_tolerance;
        limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
        limits.prune_plan = config.prune_plan;
        limits.trans_stopped_vel = config.trans_stopped_vel;
        limits.theta_stopped_vel = config.theta_stopped_vel;

        planner_util_.reconfigureCB(limits, config.restore_defaults);

        me_->reconfigure(config);
    }

    bool CollvoidLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //when we get a new plan, we also want to clear any latch we may have on goal tolerances
        latchedStopRotateController_.resetLatching();
        current_waypoint_ = 0;
        ROS_INFO("[COLLVOID] Got new plan");
        return planner_util_.setPlan(orig_global_plan);
    }

    bool CollvoidLocalPlanner::isGoalReached()
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
        {
            ROS_INFO("Goal reached");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool CollvoidLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        transformed_plan_.clear();
        if (!planner_util_.getLocalPlan(current_pose_, transformed_plan_))
        {
            ROS_ERROR("[Collvoid local planner] Could not get local plan");
            return false;
        }

        //if the global plan passed in is empty... we won't do anything
        if (transformed_plan_.empty())
        {
            ROS_WARN_NAMED("collvoid_local_planner", "Received an empty transformed plan.");
            return false;
        }

        // Set current velocities from odometry
        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        me_->updatePlanAndLocalCosts(current_pose_, transformed_plan_, robot_vel);
        //check to see if we've reached the goal position
        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
        {
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            return latchedStopRotateController_.computeVelocityCommandsStopRotate(cmd_vel,
                                                                                  limits.getAccLimits(),
                                                                                  me_->sim_period_,
                                                                                  &planner_util_,
                                                                                  odom_helper_,
                                                                                  current_pose_,
                                                                                  boost::bind(&ROSAgent::checkTrajectory, me_, _1, _2, _3));
        }

        geometry_msgs::PoseStamped target_pose;

        // target_pose.setIdentity();
        target_pose.header.frame_id = planner_util_.getGlobalFrame(); //TODO should be base frame?
        if (!skip_next_)
        {
            geometry_msgs::PoseStamped target_pose_msg;
            findBestWaypoint(target_pose_msg, current_pose_);
        }
        tf2::convert(transformed_plan_.at(current_waypoint_), target_pose);

        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped pos;

        tf2::convert(current_pose_, pos);
        local_plan.push_back(pos);
        geometry_msgs::PoseStamped pos2 = transformed_plan_.at(current_waypoint_);
        local_plan.push_back(pos2);
        publishGlobalPlan(transformed_plan_);
        publishLocalPlan(local_plan);

        geometry_msgs::PoseStamped me_pose;

        tf_->transform(current_pose_, me_pose, me_->global_frame_);
        tf_->transform(target_pose, target_pose, me_->global_frame_);

        geometry_msgs::Twist res;

        res.linear.x = target_pose.pose.position.x - me_pose.pose.position.x;
        res.linear.y = target_pose.pose.position.y - me_pose.pose.position.y;
        //res.angular.z = angles::shortest_angular_distance(tf::getYaw(me_pose.getRotation()), atan2(res.linear.y, res.linear.x));  //atan2 ==> arctangens, returns angle of polar coordinates
        tf2::Quaternion q_target;
        tf2::Quaternion q_me;
        tf2::convert(target_pose.pose.orientation, q_target);
        tf2::convert(me_pose.pose.orientation, q_me);
        res.angular.z = q_me.angleShortestPath(q_target); // TODO maybe the other way around

        collvoid::Vector2 goal_dir = collvoid::Vector2(res.linear.x, res.linear.y);
        // collvoid::Vector2 goal_dir = collvoid::Vector2(goal_x,goal_y);

        if (collvoid::abs(goal_dir) > planner_util_.getCurrentLimits().max_vel_trans)
        {
            goal_dir = planner_util_.getCurrentLimits().max_vel_trans * collvoid::normalize(goal_dir);
        }

        else if (collvoid::abs(goal_dir) < planner_util_.getCurrentLimits().min_vel_trans)
        {
            goal_dir = planner_util_.getCurrentLimits().min_vel_trans * 1.2 * collvoid::normalize(goal_dir);
        }

        collvoid::Vector2 pref_vel = collvoid::Vector2(goal_dir.x(), goal_dir.y());

        me_->computeNewVelocity(pref_vel, cmd_vel);

        double cmd_vel_len = collvoid::abs(collvoid::Vector2(cmd_vel.linear.x, cmd_vel.linear.y));
        if (std::abs(cmd_vel.angular.z) < planner_util_.getCurrentLimits().min_vel_theta)
            if (std::abs(cmd_vel.angular.z) > planner_util_.getCurrentLimits().min_vel_theta / 2)
                cmd_vel.angular.z = sign(cmd_vel.angular.z) * planner_util_.getCurrentLimits().min_vel_theta;
            else
                cmd_vel.angular.z = 0.0;

        return !(cmd_vel.angular.z == 0.0 && cmd_vel_len < planner_util_.getCurrentLimits().min_vel_trans);
    }

    void CollvoidLocalPlanner::findBestWaypoint(geometry_msgs::PoseStamped &target_pose,
                                                const geometry_msgs::PoseStamped &global_pose)
    {
        //current_waypoint_ = 0;
        current_waypoint_ = 0;
        double min_dist = DBL_MAX;
        for (size_t i = current_waypoint_; i < transformed_plan_.size(); i++)
        {
            //double y = global_pose.getOrigin().y();
            //double x = global_pose.getOrigin().x();
            double dist = base_local_planner::getGoalPositionDistance(global_pose, transformed_plan_.at(i).pose.position.x,
                                                                      transformed_plan_.at(i).pose.position.y);
            if (dist < me_->fixed_robot_radius_ || dist < min_dist)
            {
                min_dist = dist;
                target_pose = transformed_plan_.at(i);
                current_waypoint_ = i;
            }
        }

        //ROS_INFO("waypoint = %d, of %d", current_waypoint_, (int)transformed_plan_.size());

        if (current_waypoint_ == transformed_plan_.size() - 1) //I am at the end of the plan
            return;

        double dif_x = transformed_plan_.at(current_waypoint_ + 1).pose.position.x - target_pose.pose.position.x;
        double dif_y = transformed_plan_.at(current_waypoint_ + 1).pose.position.y - target_pose.pose.position.y;

        double plan_dir = atan2(dif_y, dif_x);

        double dif_ang = plan_dir;

        //ROS_DEBUG("dif = %f,%f of %f",dif_x,dif_y, dif_ang );

        //size_t look_ahead_ind = 0;
        //bool look_ahead = false;

        for (size_t i = current_waypoint_ + 1; i < transformed_plan_.size(); i++)
        {
            dif_x = transformed_plan_.at(i).pose.position.x - target_pose.pose.position.x;
            dif_y = transformed_plan_.at(i).pose.position.y - target_pose.pose.position.y;

            dif_ang = atan2(dif_y, dif_x);
            //target_pose = transformed_plan_[current_waypoint_];

            if (fabs(plan_dir - dif_ang) > 1.0 * planner_util_.getCurrentLimits().yaw_goal_tolerance)
            {
                target_pose = transformed_plan_.at(i - 1);
                current_waypoint_ = i - 1;
                //ROS_DEBUG("waypoint = %d, of %d", current_waypoint_, transformed_plan_.size());

                return;
            }
        }
        target_pose = transformed_plan_.back();
        current_waypoint_ = transformed_plan_.size() - 1;
    }

    void CollvoidLocalPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void CollvoidLocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }

}; // namespace collvoid_local_planner
