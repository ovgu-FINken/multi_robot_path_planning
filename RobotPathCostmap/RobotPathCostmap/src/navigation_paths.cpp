#include <string>
#include <nav_msgs>
#include <list>
#include <RobotPathCostmap/navigation_paths.h>

//http://docs.ros.org/api/nav_msgs/html/msg/Path.html


namespace navigation_path_layers 
{

void NavigationPathLayer::onInitialize()
{

    bool* side_inflation = false;
    float* inflation_size = 1;
    ros::NodeHandle nh("~/" + name_), g_nh; // ToDo: check cooperation with other Multi-robot-path-planning-files
    paths_sub_ = nh.subscribe("/paths", 1, &NavigationPathLayer::pathCallback, this); // ToDo: check if frame_id of robot stays the same
    list<Path> paths_list_;

}

void NavigationPathLayer::pathCallback(const nav_msgs::Path& path)
{
    boost::recursive_mutex::scoped_lock lock(lock_);

    /* list handling: if new path or update of existing robots path 
        --> if existing delete old costmap manipulation and add new at same position
        --> if new push to list and push id/name of robot to list
    */

    bool* isOld = false;
    unsigned int* index_ = paths_list_.size(); // this value cannot be assigned in for-loop (as back check)

    for (unsigned int i = 0; i < paths_list_.size(); i++)
    {
        nav_msgs::Path& path_ = paths_list_[i];
        if (path_.header.frame_id.compare(path.header.frame_id) == 0)
        {
            isOld = true;
            index_ = i;
            break;
        }
    }

    if (paths_list_.size == 0 || isOld)
    {
        paths_list_.push(paths);
    } else
    {
        paths_list_[i] = path;
    }

    delete isOld;
    delete index_;
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

void NavigationPathLayer::scaleSideInflation(float* inflation_scale)
{
    inflation_size* = inflation_scale;
}

void NavigationPathLayer::inflate_side()
{
// optional
// create costs on other robots' right side to always pass them on the other


}

}

PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)