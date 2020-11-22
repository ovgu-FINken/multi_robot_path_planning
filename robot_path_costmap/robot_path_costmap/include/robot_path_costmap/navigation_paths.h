#ifndef NAVIGATION_PATHS_LAYER_H
#define NAVIGATION_PATHS_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <robot_path_costmap/NavigationPathLayerConfig.h>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>


using namespace std;

namespace navigation_path_layers {

	static double gauss_sigma; 
	static double gauss_r;
	static double gauss_s;

	class NavigationPathLayer : public costmap_2d::Layer
	{
		static const int MAX_FILTER_SIZE = 25;
		static tf2_ros::TransformBroadcaster br;
		// ToEnhance: Read from file (avoid hard code)
		static constexpr double res = 0.05; 

	public:
		NavigationPathLayer()
		{
			layered_costmap_ = NULL;
		}
		virtual void onInitialize();
		virtual void pathCallback_l(const nav_msgs::Path& paths);
		virtual void pathCallback_g(const nav_msgs::Path& paths);
		virtual void pathCallback(const nav_msgs::Path& paths, const bool isGlobal);
		virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y);
		virtual void updateCosts_();
		virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
		// ******** NOT IMPLEMENTED YET ***************************
		// virtual void setSideInflation(bool inflate);
		// virtual void setFilterSize(int size);
		// virtual void scaleSideInflation(double inflation_scale);
		// virtual void setFilterStrength(int s);

	protected:
		bool first_time_;

		// ToEnhance: set the number of subscribers to the double of robots used (avoid hard code)
		ros::Subscriber paths_sub[16];

		boost::recursive_mutex lock_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>* server_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>::CallbackType f_;

		double filter_strength;
		int filter_size;
		int number_of_future_steps;
		int max_number_of_future_steps;
		bool side_inflation;
		double inflation_strength;
		double kernel[MAX_FILTER_SIZE][MAX_FILTER_SIZE];
		double gauss_sigma, gauss_s, gauss_r;

  		double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

		list<nav_msgs::Path> paths_list_g;
		list<nav_msgs::Path> paths_list_l;
		nav_msgs::Path path_;

		virtual costmap_2d::Costmap2D createCostHillChain(list<vector<double>> positions, costmap_2d::Costmap2D costmap, const string frame);
		virtual void createFilter();
		virtual costmap_2d::Costmap2D useFilter(std::vector<double> position, costmap_2d::Costmap2D costmap, int pos);
		void configure(robot_path_costmap::NavigationPathLayerConfig &config, uint32_t level);
		virtual void resetCosts(costmap_2d::Costmap2D costmap);
		vector<double> transform(geometry_msgs::PoseStamped pose_, const string frame);
		vector<double> getTransform(const string frame, const string origin);
		// ******** NOT IMPLEMENTED YET ***************************
		// virtual costmap_2d::Costmap2D useSideFilter(std::vector<int> position, costmap_2d::Costmap2D costmap, double downward_scale);
	};
}

#endif  // NAVIGATION_PATHS_LAYER_H
