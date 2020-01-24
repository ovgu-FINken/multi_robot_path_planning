#include <string>
#include <RobotPathCostmap/navigation_paths.h>


namespace navigation_path_layers 
{

void NavigationPathLayer::onInitialize()
{

    bool* side_inflation = false;

}

void NavigationPathLayer::updateBounds()
{

}

void NavigationPathLayer::updateCosts()
{

}

void NavigationPathLayer::setSideInflation(bool* inflate)
{
    side_inflation* = inflate;
}

void NavigationPathLayer::inflate_side()
{
// optional
// create costs on other robots' right side to always pass them on the other


}

}