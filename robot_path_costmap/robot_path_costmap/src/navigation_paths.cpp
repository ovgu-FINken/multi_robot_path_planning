
#include "../include/robot_path_costmap/navigation_paths.h"
#include <iterator>
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)

using namespace std;
// using namespace navigation_path_layers;

/*
    TODOS:
    - NavigationPathLayerConfig unbekannt
    - Funktion, um poses in String zu verwandeln / zu vergleichen
    - Listenwert updaten (pathCallback)

*/

namespace navigation_path_layers 
{

void NavigationPathLayer::onInitialize()
{
    first_time_ = true;
    ros::NodeHandle nh("~/" + name_), g_nh;
	server_ = new dynamic_reconfigure::Server<NavigationPathLayerConfig>(nh);


    /*
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: In member function ‘virtual void navigation_path_layers::NavigationPathLayer::onInitialize()’:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:23:44: error: ‘NavigationPathLayerConfig’ was not declared in this scope
  server_ = new dynamic_reconfigure::Server<NavigationPathLayerConfig>(nh);
                                            ^~~~~~~~~~~~~~~~~~~~~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:23:44: note: suggested alternative:
In file included from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:13:0,
                 from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:2:
/home/pathplanning/DrivingSwarm/devel/.private/robot_path_costmap/include/robot_path_costmap/NavigationPathLayerConfig.h:32:9: note:   ‘robot_path_costmap::NavigationPathLayerConfig’
   class NavigationPathLayerConfig
         ^~~~~~~~~~~~~~~~~~~~~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:23:69: error: template argument 1 is invalid
  server_ = new dynamic_reconfigure::Server<NavigationPathLayerConfig>(nh);
                                                                     ^
    */

    
	f_ = boost::bind(&NavigationPathLayer::configure, this, _1, _2);
	server_->setCallback(f_);
    paths_sub_ = nh.subscribe("/local_plan", 1, &NavigationPathLayer::pathCallback, this);

	NavigationPathLayer::createFilter();

}

void NavigationPathLayer::pathCallback(const nav_msgs::Path& path) // ToDo: CHECK!!!!!!!!!!
{
    boost::recursive_mutex::scoped_lock lock(lock_);

    /* list handling: if new path or update of existing robots path 
        --> if existing delete old costmap manipulation and add new at same position
        --> if new push to list and push id/name of robot to list
    */

    bool isOld = false;
    bool changed = false;
    int index_; // this value cannot be assigned in for-loop (as back check)

    for (int i = 0; i < paths_list_.size(); i++)
    {   
        list<nav_msgs::Path>::iterator it = next(paths_list_.begin(), i); 
        nav_msgs::Path& path_ = *it;
        if (path_.header.frame_id.compare(path.header.frame_id) == 0)
        {
            isOld = true;
            index_ = i;
            break;
        }
    }

    if (!isOld)
    {
		paths_list_.push_back(path);
    } else
    {
        string oldPath = string(next(paths_list_.begin(), index_).poses);
        string newPath = string(path.poses);
        if (oldPath.compare(newPath) != 0)
        {
            changed = true;
        }

        list<nav_msgs::Path>::iterator it = next(paths_list_.begin(), index_); 
        paths_list_.remove(*it);
	/* 
	paths_list_.erase(index_);
	*/

        /* ----------------------------------------------------- edited ----------------------------------------------------------------------
/usr/include/c++/7/bits/basic_string.h:420:7: note:   candidate expects 0 arguments, 1 provided
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:70:30: error: no matching function for call to ‘std::__cxx11::list<nav_msgs::Path_<std::allocator<void> > >::remove(std::__cxx11::list<nav_msgs::Path_<std::allocator<void> > >::iterator&)’
         paths_list_.remove(it);
                              ^
In file included from /usr/include/c++/7/list:64:0,
                 from /opt/ros/melodic/include/ros/forwards.h:35,
                 from /opt/ros/melodic/include/ros/common.h:37,
                 from /opt/ros/melodic/include/ros/ros.h:43,
                 from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:3,
                 from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:2:
/usr/include/c++/7/bits/list.tcc:324:5: note: candidate: void std::__cxx11::list<_Tp, _Alloc>::remove(const value_type&) [with _Tp = nav_msgs::Path_<std::allocator<void> >; _Alloc = std::allocator<nav_msgs::Path_<std::allocator<void> > >; std::__cxx11::list<_Tp, _Alloc>::value_type = nav_msgs::Path_<std::allocator<void> >]
     list<_Tp, _Alloc>::
     ^~~~~~~~~~~~~~~~~
        */

		paths_list_.push_back(path); // timestamp is newer
    }

    if (changed ||!isOld)
    {
		NavigationPathLayer::updateCosts();
    }
}



void NavigationPathLayer::updateBounds( double* min_x, double* min_y, double* max_x, double* max_y)
{
    // Grenzen der Costmap neu setzen, um alle benötigten Punkte zu beinhalten
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
	if (!enabled_) return;

    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    // function call only if given path changed
    // reset costs to 0
    // resetCosts();

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


/* void NavigationPathLayer::resetCosts()
{
    // gesamte Layer auf 0 zurücksetzen
} */

costmap_2d::Costmap2D NavigationPathLayer::createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D costmap) // Pfad übergeben
/*
/usr/include/c++/7/bits/list.tcc:324:5: note:   no known conversion for argument 1 from ‘std::__cxx11::list<nav_msgs::Path_<std::allocator<void> > >::iterator {aka std::_List_iterator<nav_msgs::Path_<std::allocator<void> > >}’ to ‘const value_type& {aka const nav_msgs::Path_<std::allocator<void> >&}’
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: In member function ‘virtual void navigation_path_layers::NavigationPathLayer::updateCosts()’:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:161:57: error: no matching function for call to ‘navigation_path_layers::NavigationPathLayer::createCostHillChain(std::__cxx11::list<std::vector<int> >&, costmap_2d::Costmap2D*&)’
         costmap = createCostHillChain(positions, costmap);
                                                         ^
In file included from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:2:0:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:62:33: note: candidate: virtual costmap_2d::Costmap2D navigation_path_layers::NavigationPathLayer::createCostHillChain(std::__cxx11::list<std::vector<int> >, costmap_2d::Costmap2D&)
   virtual costmap_2d::Costmap2D createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D& costmap);
                                 ^~~~~~~~~~~~~~~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:62:33: note:   no known conversion for argument 2 from ‘costmap_2d::Costmap2D*’ to ‘costmap_2d::Costmap2D&’
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: At global scope:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:204:23: error: prototype for ‘costmap_2d::Costmap2D navigation_path_layers::NavigationPathLayer::createCostHillChain(std::__cxx11::list<std::vector<int> >, costmap_2d::Costmap2D)’ does not match any in class ‘navigation_path_layers::NavigationPathLayer’
 costmap_2d::Costmap2D NavigationPathLayer::createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D costmap) // Pfad übergeben
                       ^~~~~~~~~~~~~~~~~~~
In file included from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:2:0:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:62:33: error: candidate is: virtual costmap_2d::Costmap2D navigation_path_layers::NavigationPathLayer::createCostHillChain(std::__cxx11::list<std::vector<int> >, costmap_2d::Costmap2D&)
   virtual costmap_2d::Costmap2D createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D& costmap);
                                 ^~~~~~~~~~~~~~~~~~~

*/
{
	costmap_2d::Costmap2D costmap_ = costmap;
    // increase costs along the path
	for (int pos = 0; pos < positions.size(); pos++)
	{
		vector<int> position = *next(positions.begin(), pos);
		costmap_ = useFilter(position, costmap_);
	}
 
	// add later
    // entlang des Pfades linear abnehmende/zunehmende Kosten? (je weiter in die "Zukunft" desto weniger /mehr Einfluss auf die Costmap 
    // (Unsicherheit als geringe Wahrscheinlichkeit oder als größeren Puffer betrachten))
	// filterstrength hier auslesen, linear interpolieren und Ergebnis an useFilter übergeben, statt dort erst auszulesen

    // open-cv gaußfilter costmap konvertieren ggfs
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

    return costmap_;
}

void NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
{
	// sum is for normalization 
	double sum = 0.0;
	int bound = int((filter_size - 1) / 2);
	int buffer = int((NavigationPathLayer::MAX_FILTER_SIZE - filter_size) / 2);

	for (int i = -bound; i <= bound; i++)
	{
		for (int j = -bound; j <= bound; j++)
		{
			navigation_path_layers::gauss_r = sqrt(i * i + j * j);
			kernel[i + bound + buffer][j + bound + buffer] = (exp(-(navigation_path_layers::gauss_r * navigation_path_layers::gauss_r) / navigation_path_layers::gauss_s)) / (M_PI * navigation_path_layers::gauss_s);
			sum += kernel[i + bound + buffer][j + bound + buffer];
            /* ----------------------------------------------------- edited ----------------------------------------------------------------------
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: In member function ‘virtual void navigation_path_layers::NavigationPathLayer::createFilter()’:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:258:4: error: ‘kernel’ was not declared in this scope
    kernel[i + bound + buffer][j + bound + buffer] = (exp(-(navigation_path_layers::gauss_r * navigation_path_layers::gauss_r) / navigation_path_layers::gauss_s)) / (M_PI * navigation_path_layers::gauss_s);
    ^~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:258:4: note: suggested alternative: ‘erfl’
    kernel[i + bound + buffer][j + bound + buffer] = (exp(-(navigation_path_layers::gauss_r * navigation_path_layers::gauss_r) / navigation_path_layers::gauss_s)) / (M_PI * navigation_path_layers::gauss_s);
    ^~~~~~
    erfl
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:268:4: error: ‘kernel’ was not declared in this scope
    kernel[i][j] /= sum;
    ^~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:268:4: note: suggested alternative: ‘erfl’
    kernel[i][j] /= sum;
    ^~~~~~
    erfl
            */
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
			// double current = costmap[position[0] + i][position[1] + j];
			double current = costmap->getCost(position[0] + i, position[1] + j);
            /*
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: In member function ‘virtual costmap_2d::Costmap2D navigation_path_layers::NavigationPathLayer::useFilter(std::vector<int>, costmap_2d::Costmap2D)’:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:303:28: error: no match for ‘operator[]’ (operand types are ‘costmap_2d::Costmap2D’ and ‘__gnu_cxx::__alloc_traits<std::allocator<int> >::value_type {aka int}’)
    double current = costmap[position[0] + i][position[1] + j];
                            ^
            */
			// _map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * filter_strength;
			_map->setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * filter_strength));
            /*
            /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:304:8: error: no match for ‘operator[]’ (operand types are ‘costmap_2d::Costmap2D’ and ‘__gnu_cxx::__alloc_traits<std::allocator<int> >::value_type {aka int}’)
    _map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * filter_strength);
        ^
	
	----------------------------------------------------- edited ----------------------------------------------------------------------
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:304:58: error: ‘kernel’ was not declared in this scope
    _map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * filter_strength);
                                                          ^~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:304:58: note: suggested alternative: ‘erfl’
    _map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * filter_strength);
                                                          ^~~~~~
                                                          erfl

            */
		}
	}

/*
	if (side_inflation)
	{
		// ToDo:
		vector<int> side = position;
		// orientation einarbeiten
		// add values to "right" side of robot
		side[0] = position[0] + filter_size * x + position[1] + filter_size * y;
		side[1] = position[1] + filter_size * x + position[0] + filter_size * y;

		_map = NavigationPathLayer::useSideFilter(side, _map);
	}*/

    return _map;
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
			// double current = costmap[position[0] + i][position[1] + j];
			double current = costmap->getCost(position[0] + i, position[1] + j);
            /*
            entsprechend useFilter
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: In member function ‘virtual costmap_2d::Costmap2D navigation_path_layers::NavigationPathLayer::useSideFilter(std::vector<int>, costmap_2d::Costmap2D)’:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:334:28: error: no match for ‘operator[]’ (operand types are ‘costmap_2d::Costmap2D’ and ‘__gnu_cxx::__alloc_traits<std::allocator<int> >::value_type {aka int}’)
    double current = costmap[position[0] + i][position[1] + j];
                            ^
            */
			// _map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * inflation_strength);
			_map->setCost(position[0] + i, position[1] + j, max(current, kernel[i + buffer][j + buffer] * inflation_strength));
            /*
            entsprechend useFilter
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:335:8: error: no match for ‘operator[]’ (operand types are ‘costmap_2d::Costmap2D’ and ‘__gnu_cxx::__alloc_traits<std::allocator<int> >::value_type {aka int}’)
    _map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * inflation_strength);
        ^
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:335:58: error: ‘kernel’ was not declared in this scope
    _map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * inflation_strength);
                                                          ^~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:335:58: note: suggested alternative: ‘erfl’
    _map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * inflation_strength);
                                                          ^~~~~~
                                                          erfl
            */
		}
	}

	return _map;
}

void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
/*
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp: At global scope:
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:37: error: variable or field ‘configure’ declared void
 void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
                                     ^~~~~~~~~~~~~~~~~~~~~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:37: error: ‘NavigationPathLayerConfig’ was not declared in this scope
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:37: note: suggested alternative:
In file included from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/../include/robot_path_costmap/navigation_paths.h:13:0,
                 from /home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:2:
/home/pathplanning/DrivingSwarm/devel/.private/robot_path_costmap/include/robot_path_costmap/NavigationPathLayerConfig.h:32:9: note:   ‘robot_path_costmap::NavigationPathLayerConfig’
   class NavigationPathLayerConfig
         ^~~~~~~~~~~~~~~~~~~~~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:64: error: ‘config’ was not declared in this scope
 void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
                                                                ^~~~~~
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:64: note: suggested alternative: ‘conjq’
 void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
                                                                ^~~~~~
                                                                conjq
/home/pathplanning/DrivingSwarm/src/pathplanning/robot_path_costmap/robot_path_costmap/src/navigation_paths.cpp:342:81: error: expected primary-expression before ‘level’
 void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
                                                                                 ^~~~~
*/
{
	filter_strength = config.filter_strength;
	filter_size = config.filter_size;
	side_inflation = config.side_inflation;
	inflation_strength = config.inflation_strength;
	gauss_sigma = config.gauss_sigma;
	gauss_s = config.gauss_s * gauss_sigma * gauss_sigma;
	enabled_ = config.enabled;
}

}
