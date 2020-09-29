
#include "../include/robot_path_costmap/navigation_paths.h"

using namespace std;
// using namespace navigation_path_layers;



namespace navigation_path_layers 
{

    /*
	static const double gauss_sigma = 1.0;
	static const double gauss_r, gauss_s = 2.0 * gauss_sigma * gauss_sigma;
	*/

void NavigationPathLayer::onInitialize()
{

    /* 
    side_inflation = false;
	filter_strength = 1; 
    inflation_strength = 0.05;
    filter_size = 9; // resolution: 0.050000 meters / pixel ; 4 pixel needed for robot itself
	*/
    first_time_ = true;
    ros::NodeHandle nh("~/" + name_), g_nh;
	server_ = new dynamic_reconfigure::Server<NavigationPathLayerConfig>(nh);
	f_ = boost::bind(&NavigationPathLayer::configure, this, _1, _2);
	server_->setCallback(f_);
    paths_sub_ = nh.subscribe("/local_plan", 1, &NavigationPathLayer::pathCallback, this);

	kernel = &NavigationPathLayer::createFilter();

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
    unsigned int index_; // this value cannot be assigned in for-loop (as back check)

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

    if (!isOld)
    {
		paths_list_.push_back(path);
    } else
    {
        string oldPath = string(paths_list_[index_].poses);
        string newPath = string(path.poses);
        if (oldPath.compare(newPath) != 0)
        {
            changed = true;
        }

		paths_list_[index_] = path; // timestamp is newer
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
            positions.push_back(position)
        }
        // add cost hills per path
        costmap = createCostHillChain(positions, &costmap);
    }
}

void NavigationPathLayer::setSideInflation(bool inflate)
{
	side_inflation = inflate;
    kernel = NavigationPathLayer::createFilter();
}

void NavigationPathLayer::scaleSideInflation(double inflation_scale)
{
    // maximal factor 1 of the costs of the normal path
    inflation_strength = max(min(inflation_scale, 1.0), 0.0);
	kernel = NavigationPathLayer::createFilter();
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
    
    kernel = NavigationPathLayer::createFilter();
}

void NavigationPathLayer::setFilterStrength(int s)
{
    filter_strength = s;
    kernel = NavigationPathLayer::createFilter();
}


/* void NavigationPathLayer::resetCosts()
{
    // gesamte Layer auf 0 zurücksetzen
} */

costmap_2d::Costmap2D NavigationPathLayer::createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D& costmap) // Pfad übergeben
{
	costmap_2d::Costmap2D costmap_ = costmap;
    // increase costs along the path
	for (unsigned int pos = 0; pos < positions.size(); i++)
	{
		vector<int> position = positions[pos];
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

double[MAX_FILTER_SIZE][MAX_FILTER_SIZE] NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
{
	// sum is for normalization 
	double sum = 0.0;
	int bound = int((filter_size - 1) / 2);
	int buffer = int((NavigationPathLayer::MAX_FILTER_SIZE - filter_size) / 2);
	double[MAX_FILTER_SIZE][MAX_FILTER_SIZE] kernel;

	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
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

	return kernel;
}

costmap_2d::Costmap2D NavigationPathLayer::useFilter(vector<int> position, costmap_2d::Costmap2D& costmap)
{
	int bound = int((filter_size - 1) / 2);
	int buffer = int((MAX_FILTER_SIZE - 1) / 2);
	costmap_2d::Costmap2D _map = costmap;

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			double current = costmap[position[0] + i][position[1] + j];
			_map[position[0] + i][position[1] + j] = max(current, kernel[i + buffer][j + buffer] * filter_strength);
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

costmap_2d::Costmap2D NavigationPathLayer::useSideFilter(vector<int> position, costmap_2d::Costmap2D& costmap)
{
	int bound = int((filter_size - 1) / 2);
	costmap_2d::Costmap2D _map = costmap;

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			double current = costmap[position[0] + i][position[1] + j];
			_map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * inflation_strength);
		}
	}

	return _map;
}

void NavigationPathLayer::configure(NavigationPathLayerConfig &config, uint32_t level)
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

PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)
