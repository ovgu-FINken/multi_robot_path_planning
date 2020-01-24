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
    virtual void NavigationPathLayer::updateBounds();
    virtual void NavigationPathLayer::updateCosts();
    virtual void NavigationPathLayer::setSideInflation(bool* inflate);

protected:

    virtual void NavigationPathLayer::inflate_side();

private:
    bool* side_inflation;


}

#endif  // NAVIGATION_PATHS_LAYER_H