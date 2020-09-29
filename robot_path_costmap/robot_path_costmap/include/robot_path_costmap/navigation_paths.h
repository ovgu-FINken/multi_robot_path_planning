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
#include <robot_path_costmap/NavigationPathLayerConfig.h>

namespace navigation_path_layers {

	static double gauss_sigma; 
	static double gauss_r;
	static double gauss_s;

	class NavigationPathLayer : public costmap_2d::Layer
	{
	public:
		NavigationPathLayer()
		{
			layered_costmap_ = NULL;
		}
		virtual void onInitialize();
		virtual void pathCallback(const nav_msgs::Path& paths);
		virtual void updateBounds(double min_x, double min_y,
			double max_x, double max_y);
		virtual void updateCosts();
		virtual void setSideInflation(bool inflate);
		virtual void setFilterSize(int size);
		virtual void scaleSideInflation(double inflation_scale);
		virtual void setFilterStrength(int s);

	protected:
		bool first_time_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>* server_;
		dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>::CallbackType f_;

	private:
		double filter_strength;
		int filter_size;
		bool side_inflation;
		double inflation_strength;
		double[][] kernel;
		double gauss_sigma, gauss_s, gauss_r;
		// virtual void NavigationPathLayer::resetCosts();
		virtual costmap_2d::Costmap2D* createCostHillChain(std::vector<std::vector<int>> positions, costmap_2d::Costmap2D* costmap);
		virtual double[][] createFilter();
		virtual costmap_2d::Costmap2D* useFilter(std::vector<int> position, costmap_2d::Costmap2D* costmap);
		virtual costmap_2d::Costmap2D* useSideFilter(std::vector<int> position, costmap_2d::Costmap2D* costmap);
		void configure(NavigationPathLayerConfig &config, uint32_t level);
	};
}

#endif  // NAVIGATION_PATHS_LAYER_H
