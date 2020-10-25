
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
        // for updateBounds
        first_time_ = true;
        current_ = true;

        // initialize dynamic reconfigure
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<robot_path_costmap::NavigationPathLayerConfig>(nh);    
        f_ = boost::bind(&NavigationPathLayer::configure, this, _1, _2);
        server_->setCallback(f_);
	    
	std::string ns = ros::this_node::getNamespace(); // Regex verwenden
	for (int i = 0; i < 8; i++)
	{
		// tb3_x --> x is cast to int and compared to iterator
		// if it's its own namespace, do nothing
		if (boost::lexical_cast<int>(ns[5]) != i) // ToDo: regex
		{       
			// subscribe to plans 
			paths_sub[2*i] = g_nh.subscribe("/tb3_" + to_string(i) + "/move_base/DWAPlannerROS/local_plan", 10, &NavigationPathLayer::pathCallback, this);
        	paths_sub[2*i + 1]  = g_nh.subscribe("/tb3_" + to_string(i) + "/move_base/DWAPlannerROS/global_plan", 10, &NavigationPathLayer::pathCallback, this);
		}
	}

        NavigationPathLayer::createFilter();
    }

    void NavigationPathLayer::pathCallback(const nav_msgs::Path& path)
    {    

        /* 
            list handling: if new path or update of existing robots path 
            --> if existing delete old costmap manipulation and add new at same position
            --> if new push to list and push id/name of robot to list
        */

        boost::recursive_mutex::scoped_lock lock(lock_);
        
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
        // if path-list has been edited then update costmap
        if (changed ||!isOld)
        {
            NavigationPathLayer::updateCosts_();
        }
    }
	
    vector<int> NavigationPathLayer::transform(geometry_msgs::PoseStamped pose_)
    {
	  tf2::Transform transform;
	  transform.setOrigin( tf2::Vector3(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z) );
	  tf2::Quaternion q = *new tf2::Quaternion(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	  transform.setRotation(q);
	  NavigationPathLayer::br.sendTransform(geometry_msgs::TransformStamped(transform, ros::Time::now(), "path_transform", name_));
	  return NavigationPathLayer::getTransform();
    }
	
    vector<int> NavigationPathLayer::getTransform()
    {
	    tf2_ros::TransformListener::TransformListener listener;
	    geometry_msgs::TransformStamped transform;
	    try{
	      listener.lookupTransform( "map","base_footprint",
				       ros::Time(0), transform);
	    }
	    catch (tf2::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
          return *new vector<int>{-1, -1};
	    }
	    
	    return *new vector<int>{int(transform.getOrigin().x()/NavigationPathLayer::res), int(transform.getOrigin().y()/NavigationPathLayer::res)};
    }


    void NavigationPathLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y)
    {
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

    void NavigationPathLayer::updateCosts_()
    {
        // function call only if given path changed

        // only, if plugin is activated
        if (!enabled_) return;
        costmap_2d::Costmap2D costmap = *layered_costmap_->getCostmap();

        // reset costs to 0
        NavigationPathLayer::resetCosts(costmap);

        // iterate all paths
        for(nav_msgs::Path& path: paths_list_)
        {
            list <vector<int>> positions;
            for(geometry_msgs::PoseStamped pose_ : path.poses)
            {
                vector<int> position = NavigationPathLayer::transform(pose_); // {int(pose.pose.position.x), int(pose.pose.position.y)};
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
	    number_of_future_steps = min(int(positions.size()), max_number_of_future_steps);

        cerr << "steps: " << to_string(number_of_future_steps) << "\n";

        for (int pos = 0; pos < number_of_future_steps; pos++)
        {
            vector<int> position_ = *next(positions.begin(), pos);
            cerr << "x: " << to_string(position_[0]) << "y: " << to_string(position_[1]);
        }

        cerr << "\n";

        for (int pos = 0; pos < number_of_future_steps; pos++)
        {
            vector<int> position = *next(positions.begin(), pos);
            costmap_ = useFilter(position, costmap_, pos);
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

    costmap_2d::Costmap2D NavigationPathLayer::useFilter(vector<int> position, costmap_2d::Costmap2D costmap, int pos)
    {
        int bound = int((filter_size - 1) / 2);
        int buffer = int((MAX_FILTER_SIZE - 1) / 2);
        costmap_2d::Costmap2D _map = costmap;
	    double downward_scale = (number_of_future_steps - pos) / number_of_future_steps;

        // for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
        for (int i = -bound; i <= bound; i++)
        {
            for (int j = -bound; j <= bound; j++)
            {
                if(position[0] + i >= 0 &&  position[1] + j >= 0)
                {
                    double current = costmap.getCost(position[0] + i, position[1] + j);
                    _map.setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * filter_strength * downward_scale));
                }
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

            _map = NavigationPathLayer::useSideFilter(side, _map, downward_scale);
        }*/

        return _map;
    }
	
	
    void NavigationPathLayer::resetCosts(costmap_2d::Costmap2D costmap)
    {
    	// reset layer to 0 costs
	unsigned int x_size = costmap.getSizeInCellsX();
	unsigned int y_size = costmap.getSizeInCellsY();
	    
	// from cell 0 to x_size's cell in x-direction and cell 0 to y_size's cell in y-direction
	costmap.resetMap(0, 0, x_size, y_size);
    } 

    void NavigationPathLayer::configure(robot_path_costmap::NavigationPathLayerConfig &config, uint32_t level)
    {
        filter_strength = config.filter_strength;
        filter_size = config.filter_size;
	max_number_of_future_steps = config.max_number_of_future_steps;
        side_inflation = config.side_inflation;
        inflation_strength = config.inflation_strength;
        gauss_sigma = config.gauss_sigma;
        gauss_s = config.gauss_s * gauss_sigma * gauss_sigma;
        enabled_ = config.enabled;
    }

    // https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/plugins/obstacle_layer.cpp
    void NavigationPathLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    { 
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

costmap_2d::Costmap2D NavigationPathLayer::useSideFilter(vector<int> position, costmap_2d::Costmap2D costmap, double downward_scale)
{
	int bound = int((filter_size - 1) / 2);
	costmap_2d::Costmap2D _map = costmap;

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (int i = -bound; i <= bound; i++)
	{
		for (int j = -bound; j <= bound; j++)
		{
			double current = costmap.getCost(position[0] + i, position[1] + j);
			_map.setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * inflation_strength * downward_scale));
		}
	}

	return _map;
}


*/

}
