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
    virtual void NavigationPathLayer::updateBounds();
    virtual void NavigationPathLayer::updateCosts();
    virtual void NavigationPathLayer::setSideInflation(bool inflate);
    virtual void NavigationPathLayer::setFilterSize(int size);
    virtual void NavigationPathLayer::scaleSideInflation(float* inflation_scale);
    virtual void NavigationPathLayer::setFilterStrength(float s);

protected:

    virtual void NavigationPathLayer::inflate_side();

private:
    bool* side_inflation;
    int* inflation_size;
    int* filter_size;
    virtual void NavigationPathLayer::resetCosts();
    virtual void NavigationPathLayer::createCostHillChain();
    virtual void void NavigationPathLayer::createFilter();


}

#endif  // NAVIGATION_PATHS_LAYER_H
