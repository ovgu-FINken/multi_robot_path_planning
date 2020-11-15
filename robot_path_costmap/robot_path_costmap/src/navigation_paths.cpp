#include "../include/robot_path_costmap/navigation_paths.h"
#include <iterator>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <fstream>
 
PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)

using namespace std;

namespace navigation_path_layers 
{
     tf2_ros::TransformBroadcaster NavigationPathLayer::br;


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
				paths_sub[2*i] = g_nh.subscribe("/tb3_" + to_string(i) + "/move_base/DWAPlannerROS/local_plan", 10, &NavigationPathLayer::pathCallback_l, this);
        		paths_sub[2*i + 1]  = g_nh.subscribe("/tb3_" + to_string(i) + "/move_base/DWAPlannerROS/global_plan", 10, &NavigationPathLayer::pathCallback_g, this);
			}
		}

        NavigationPathLayer::createFilter();
    }

	void NavigationPathLayer::pathCallback_g(const nav_msgs::Path& path)
	{
		NavigationPathLayer::pathCallback(path, true);
	}

	void NavigationPathLayer::pathCallback_l(const nav_msgs::Path& path)
	{
		NavigationPathLayer::pathCallback(path, false);
	}

    void NavigationPathLayer::pathCallback(const nav_msgs::Path& path, const bool isGlobal)
    {    

        /* 
            list handling: if new path or update of existing robots path 
            --> if existing delete old costmap manipulation and add new at same position
            --> if new push to list and push id/name of robot to list
        */

        boost::recursive_mutex::scoped_lock lock(lock_);
        
		// which list is used (global or local plans)
		list<nav_msgs::Path> paths_list_ = isGlobal ? paths_list_g : paths_list_l;


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

		// overwrite original list with updated version
		if (isGlobal)
		{
			paths_list_g = paths_list_;
		}
		else
		{
			paths_list_l = paths_list_;
		}

        // if path-list has been edited then update costmap
        if (changed ||!isOld)
        {
            NavigationPathLayer::updateCosts_();
        }
    }
	
    // vector<int> NavigationPathLayer::transform(geometry_msgs::PoseStamped pose_, const string frame)
	vector<double> NavigationPathLayer::transform(geometry_msgs::PoseStamped pose_, const string frame)
    {
	  geometry_msgs::TransformStamped transformStamped;
	  transformStamped.header.stamp = ros::Time::now();
  	  transformStamped.header.frame_id = "world";
  	  transformStamped.child_frame_id = name_;
	    
	  transformStamped.transform.translation.x = pose_.pose.position.x;
  	  transformStamped.transform.translation.y = pose_.pose.position.y;
  	  transformStamped.transform.translation.z = 0.0;
	    
	  tf2::Quaternion q = *new tf2::Quaternion(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
	    
	  transformStamped.transform.rotation.x = q.x();
  	  transformStamped.transform.rotation.y = q.y();
  	  transformStamped.transform.rotation.z = q.z();
  	  transformStamped.transform.rotation.w = q.w();
	    
	  NavigationPathLayer::br.sendTransform(transformStamped);
	    
	  return NavigationPathLayer::getTransform(frame);
    }
	
    // vector<int> NavigationPathLayer::getTransform(const string frame)
	vector<double> NavigationPathLayer::getTransform(const string frame)
    {
	    tf2_ros::Buffer tfBuffer;
 	    tf2_ros::TransformListener tfListener(tfBuffer);
	    geometry_msgs::TransformStamped transformStamped;
	    try{
			string origin = frame + "/odom";
			transformStamped = tfBuffer.lookupTransform("map", origin, ros::Time(0));
	    }
	    catch (tf2::TransformException &ex) {
      		ROS_WARN("%s",ex.what());
      		ros::Duration(1.0).sleep();
              	return *new vector<int>{-1, -1};
	    }
	    return *new vector<double>{transformStamped.transform.translation.x, transformStamped.transform.translation.y };
	    // return *new vector<int>{int(transformStamped.transform.translation.x / NavigationPathLayer::res), int(transformStamped.transform.translation.y / NavigationPathLayer::res)};
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

		// join both lists into one for further actions
		list<nav_msgs::Path> paths_list_ = paths_list_g;
		for (int index = 0; index < paths_list_l.size(); index++)
		{
			paths_list_.push_back(*next(paths_list_l.begin(), index);
		}

        // iterate all paths
        for(int index = 0; index < paths_list_.size(); index++)
        {
			nav_msgs::Path& path = *next(paths_list_.begin(), index);
            list <vector<double>> positions;

			// first entries are global plans' paths
			bool isGlobal = index < paths_list_g.size() ? true : false;
			string frame = path.header.frame_id;

			number_of_future_steps = min(int(path.poses.size()), max_number_of_future_steps);

            //for(geometry_msgs::PoseStamped pose_ : path.poses)
			for (int pos = 0; pos < number_of_future_steps; pos++)
            {
				geometry_msgs::PoseStamped pose_ = path.poses[pos];
                vector<double> position = isGlobal ? 
													*new vector<double>{ pose_.pose.position.x, pose_.pose.position.y } :
													NavigationPathLayer::transform(pose_, frame); // {int(pose.pose.position.x), int(pose.pose.position.y)};
                positions.push_back(position);
            }		

			// add cost hills per path
			costmap = createCostHillChain(positions, costmap, frame);
        }
    }

    costmap_2d::Costmap2D NavigationPathLayer::createCostHillChain(list<vector<double>> positions, costmap_2d::Costmap2D costmap, const string frame)
    {
        costmap_2d::Costmap2D costmap_ = costmap;



//region error_analysis
		if (boost::algorithm::contains(frame, "tb3_2/odom"))
		{
			cerr << "steps: " << to_string(number_of_future_steps) << "\n";

			for (int pos = 0; pos < number_of_future_steps; pos++)
			{
				vector<double> position_ = *next(positions.begin(), pos);
				cerr << "x: " << to_string(position_[0]) << "y: " << to_string(position_[1]);
			}

			cerr << "\n";
		}
       
//endregion error_analysis

        for (int pos = 0; pos < number_of_future_steps; pos++)
        {
            vector<double> position = *next(positions.begin(), pos);
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

    costmap_2d::Costmap2D NavigationPathLayer::useFilter(vector<double> position, costmap_2d::Costmap2D costmap, int pos)
    {
        int bound = int((filter_size - 1) / 2);
        int buffer = int((MAX_FILTER_SIZE - 1) / 2);
        costmap_2d::Costmap2D _map = costmap;
	    double downward_scale = (number_of_future_steps - pos) / number_of_future_steps;

        // for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
		unsigned int mx;
		unsigned int my;
		if (_map.worldToMap(position[0], position[1], mx, my)) 
		{
			for (int i = -bound; i <= bound; i++)
			{
				for (int j = -bound; j <= bound; j++)
				{
					double current = costmap.getCost(mx + i, my + j);
					_map.setCost(mx + i, my + j, max(current, kernel[i + buffer][j + buffer] * filter_strength * downward_scale));
						
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
            side[0] = mx + filter_size * x + my + filter_size * y;
            side[1] = my + filter_size * x + mx + filter_size * y;

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
