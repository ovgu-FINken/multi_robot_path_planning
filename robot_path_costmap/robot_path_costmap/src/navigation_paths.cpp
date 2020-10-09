
#include "../include/robot_path_costmap/navigation_paths.h"
#include <iterator>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <fstream>
 
PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)

using namespace std;

namespace navigation_path_layers 
{

    void NavigationPathLayer::onInitialize()
    {
        cerr << "Im Initialize-Bereich \n";
        // for updateBounds
        first_time_ = true;

        // initialize dynamic reconfigure
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>(nh);    
        f_ = boost::bind(&NavigationPathLayer::configure, this, _1, _2);
        server_->setCallback(f_);

        // subscribe to plans 
        // subsribe to own plan or to other robots plans right now??? // TODO check topics and add other robots
        paths_sub_l = nh.subscribe("/DWAPlannerROS/local_plan", 1, &NavigationPathLayer::pathCallback, this);
        paths_sub_g = nh.subscribe("/DWAPlannerROS/global_plan", 1, &NavigationPathLayer::pathCallback, this);

        cerr << "create Filter \n";

        NavigationPathLayer::createFilter();

        cerr << "created Filter \n";
    }

    void NavigationPathLayer::pathCallback(const nav_msgs::Path& path) // ToDo: CHECK!!!!!!!!!!
    {    
        cerr << "CALLBACK ENTRY POINT \n";

        /* 
            list handling: if new path or update of existing robots path 
            --> if existing delete old costmap manipulation and add new at same position
            --> if new push to list and push id/name of robot to list
        */

        boost::recursive_mutex::scoped_lock lock(lock_);
        ros::Time begin = ros::Time::now();
        cerr << "Einstiegszeit: " << begin << "\n";
        
        bool isOld = false;
        bool changed = false;
        int index_;

        for (int i = 0; i < paths_list_.size(); i++)
        {   
            list<nav_msgs::Path>::iterator it = next(paths_list_.begin(), i); 
            path_ = *it;
            if (path_.header.frame_id.compare(path.header.frame_id) == 0)
            {
                isOld = true;
                index_ = i;
                break;
            }
        }

        if (!isOld)
        {
            // new path is directly added to the list
            paths_list_.push_back(path);
        } else
        {
            string oldPath = path_.poses[0].header.frame_id;
            string newPath = path.poses[0].header.frame_id;

            // if changed
            if (oldPath.compare(newPath) != 0)
            {
                changed = true;
                // remove old path and add new path
                list<nav_msgs::Path>::iterator it = next(paths_list_.begin(), index_); 
                paths_list_.erase(it);
                paths_list_.push_back(path);
            }

        }
        ros::Time mid = ros::Time::now();
        cerr << "Vor updatecosts: " << mid << "\n";

        // if path-list has been edited then update costmap
        if (changed ||!isOld)
        {
            NavigationPathLayer::updateCosts();
        }
        ros::Time end = ros::Time::now();
        cerr << "Ausstiegszeit: " << end << "\n";
    }

    void NavigationPathLayer::updateBounds( double* min_x, double* min_y, double* max_x, double* max_y)
    {
        cerr << "Update bounds \n";

        // set area bounds of costmap again that is manipulated by updateCosts
        if (first_time_)
    {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        first_time_ = false;
    }
    else
    {
        double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
        *min_x = min(last_min_x_, *min_x);
        *min_y = min(last_min_y_, *min_y);
        *max_x = max(last_max_x_, *max_x);
        *max_y = max(last_max_y_, *max_y);
        last_min_x_ = a;
        last_min_y_ = b;
        last_max_x_ = c;
        last_max_y_ = d;
    }
    }

    void NavigationPathLayer::updateCosts()
    {
        // function call only if given path changed

        // only, if plugin is activated
        if (!enabled_) return;

        costmap_2d::Costmap2D costmap = *layered_costmap_->getCostmap();

        // reset costs to 0
        // resetCosts();

        // iterate all paths
        for(nav_msgs::Path& path: paths_list_)
        {
            list <vector<int>> positions;
            for(geometry_msgs::PoseStamped pose_ : path.poses)
            {
                vector<int> position = {int(pose_.pose.position.x), int(pose_.pose.position.y)};
                positions.push_back(position);
            }
            // add cost hills per path
            costmap = createCostHillChain(positions, costmap);
        }
    }

    costmap_2d::Costmap2D NavigationPathLayer::createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D costmap)
    {
        costmap_2d::Costmap2D costmap_ = costmap;
        // increase costs along the path
        for (int pos = 0; pos < positions.size(); pos++)
        {
            vector<int> position = *next(positions.begin(), pos);
            costmap_ = useFilter(position, costmap_);
        }

        return costmap_;
    }

    void NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
    {
        // sum used for normalization 
        double sum = 0.0;

        // bound as distance of actual kernel from center point
        int bound = int((filter_size - 1) / 2);
        // buffer as difference of maximum kernel size and current size
        int buffer = int((NavigationPathLayer::MAX_FILTER_SIZE - filter_size) / 2);

        for (int i = -bound; i <= bound; i++)
        {
            for (int j = -bound; j <= bound; j++)
            {
                navigation_path_layers::gauss_r = sqrt(i * i + j * j);
                kernel[i + bound + buffer][j + bound + buffer] = (exp(-(navigation_path_layers::gauss_r * navigation_path_layers::gauss_r) / navigation_path_layers::gauss_s)) / (M_PI * navigation_path_layers::gauss_s);
                sum += kernel[i + bound + buffer][j + bound + buffer];
            }
        }

        // normalising the Kernel 
        for (int i = 0; i < MAX_FILTER_SIZE; ++i)
        {
            for (int j = 0; j < MAX_FILTER_SIZE; ++j)
            {
                kernel[i][j] /= sum;
            }
        }

        return;
    }

    costmap_2d::Costmap2D NavigationPathLayer::useFilter(vector<int> position, costmap_2d::Costmap2D costmap)
    {
        int bound = int((filter_size - 1) / 2);
        int buffer = int((MAX_FILTER_SIZE - 1) / 2);
        costmap_2d::Costmap2D _map = costmap;

        // for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
        for (int i = -bound; i <= bound; i++)
        {
            for (int j = -bound; j <= bound; j++)
            {
                double current = costmap.getCost(position[0] + i, position[1] + j);
                _map.setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * filter_strength));
            }
        }

    /*
        if (side_inflation)
        {
            // ToDo:
            vector<int> side = position;
            // use orientation of robot
            // add values to "right" side of robot
            side[0] = position[0] + filter_size * x + position[1] + filter_size * y;
            side[1] = position[1] + filter_size * x + position[0] + filter_size * y;

            _map = NavigationPathLayer::useSideFilter(side, _map);
        }*/

        return _map;
    }

    void NavigationPathLayer::configure(robot_path_costmap::NavigationPathLayerConfig &config, uint32_t level)
    {
        filter_strength = config.filter_strength;
        filter_size = config.filter_size;
        side_inflation = config.side_inflation;
        inflation_strength = config.inflation_strength;
        gauss_sigma = config.gauss_sigma;
        gauss_s = config.gauss_s * gauss_sigma * gauss_sigma;
        enabled_ = config.enabled;
    }

/*
void NavigationPathLayer::setSideInflation(bool inflate)
{
	side_inflation = inflate;
    NavigationPathLayer::createFilter();
}

void NavigationPathLayer::scaleSideInflation(double inflation_scale)
{
    // maximal factor 1 of the costs of the normal path
    inflation_strength = max(min(inflation_scale, 1.0), 0.0);
	NavigationPathLayer::createFilter();
}

void NavigationPathLayer::setFilterSize(int size)
{
    if (size%2 == 1) 
    {
		filter_size = min(size, 9);
    } else
    {
        // minimum size required
        filter_size = min(size-1, 9); // ~ 12.5 cm tolerance with degrading costs
    }
    
    NavigationPathLayer::createFilter();
}

void NavigationPathLayer::setFilterStrength(int s)
{
    filter_strength = s;
    NavigationPathLayer::createFilter();
}


void NavigationPathLayer::resetCosts()
{
    // gesamte Layer auf 0 zurücksetzen
} 


costmap_2d::Costmap2D NavigationPathLayer::useSideFilter(vector<int> position, costmap_2d::Costmap2D costmap)
{
	int bound = int((filter_size - 1) / 2);
	costmap_2d::Costmap2D _map = costmap;

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (int i = -bound; i <= bound; i++)
	{
		for (int j = -bound; j <= bound; j++)
		{
			double current = costmap.getCost(position[0] + i, position[1] + j);
			_map.setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * inflation_strength));
		}
	}

	return _map;
}


*/

}
