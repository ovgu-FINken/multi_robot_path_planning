#ifndef NAVIGATION_PATHS_LAYER_H
#define NAVIGATION_PATHS_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace navigation_path_layers {

    class NavigationPathLayer : public costmap_2d::Layer
{
public:
    NavigationPathLayer()
    {
        layered_costmap_ = NULL;
    }
    virtual void NavigationPathLayer::onInitialize();
    virtual void NavigationPathLayer::pathCallback(const nav_msgs::& paths);
    virtual void NavigationPathLayer::updateBounds( double* min_x, double* min_y, 
						    double* max_x, double* max_y);
    virtual void NavigationPathLayer::updateCosts();
    virtual void NavigationPathLayer::setSideInflation(bool inflate);
    virtual void NavigationPathLayer::setFilterSize(int size);
    virtual void NavigationPathLayer::scaleSideInflation(double* inflation_scale);
    virtual void NavigationPathLayer::setFilterStrength(int s);

protected:
    bool first_time_;
    virtual void NavigationPathLayer::inflate_side();

private:
    bool* side_inflation;
    int* inflation_size;
    int* filter_size;
    int* filter_strength;
    virtual void NavigationPathLayer::resetCosts();
    virtual costmap_2d::Costmap2D* NavigationPathLayer::createCostHillChain(std::vector<std::vector<int>> positions, costmap_2d::Costmap2D* costmap);
    virtual void NavigationPathLayer::createFilter();
    virtual costmap_2d::Costmap2D* NavigationPathLayer::useFilter(std::vector<int> position, costmap_2d::Costmap2D* costmap);


}

#endif  // NAVIGATION_PATHS_LAYER_H
