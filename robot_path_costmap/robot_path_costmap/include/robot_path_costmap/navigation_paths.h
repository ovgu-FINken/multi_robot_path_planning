#ifndef NAVIGATION_PATHS_LAYER_H
#define NAVIGATION_PATHS_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <string>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <stdint.h>
#include <robot_path_costmap/NavigationPathLayerConfig.h>
#include <iostream>

using namespace std;

namespace navigation_path_layers {

	static double gauss_sigma; 
	static double gauss_r;
	static double gauss_s;

	class NavigationPathLayer : public costmap_2d::Layer
	{
		static const int MAX_FILTER_SIZE = 25;

	public:
		NavigationPathLayer()
		{
			layered_costmap_ = NULL;
			cerr << "Markierung fuer Einstieg \n";
		}
		virtual void onInitialize();
		virtual void pathCallback(const nav_msgs::Path& paths);
		virtual void updateBounds(double* min_x, double* min_y,	double* max_x, double* max_y);
		virtual void updateCosts();
		// virtual void setSideInflation(bool inflate);
		// virtual void setFilterSize(int size);
		// virtual void scaleSideInflation(double inflation_scale);
		// virtual void setFilterStrength(int s);

	protected:
		bool first_time_;

		ros::Subscriber paths_sub_l;
		ros::Subscriber paths_sub_g;

		boost::recursive_mutex lock_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>* server_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>::CallbackType f_;

		double filter_strength;
		int filter_size;
		bool side_inflation;
		double inflation_strength;
		double kernel[MAX_FILTER_SIZE][MAX_FILTER_SIZE];
		double gauss_sigma, gauss_s, gauss_r;

  		double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

		list<nav_msgs::Path> paths_list_;
		nav_msgs::Path path_;

		virtual costmap_2d::Costmap2D createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D costmap);
		virtual void createFilter();
		virtual costmap_2d::Costmap2D useFilter(std::vector<int> position, costmap_2d::Costmap2D costmap);
		void configure(robot_path_costmap::NavigationPathLayerConfig &config, uint32_t level);
		// virtual costmap_2d::Costmap2D useSideFilter(std::vector<int> position, costmap_2d::Costmap2D costmap);
		// virtual void NavigationPathLayer::resetCosts();
	};
}

#endif  // NAVIGATION_PATHS_LAYER_H
