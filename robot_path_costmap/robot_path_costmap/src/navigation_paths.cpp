#include <string>
#include <nav_msgs>
#include <list>
#include <vector>
#include <cmath>
#include "./../include/RobotPathCostmap/navigation_paths.h">

using namespace std;




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
    list<Path> paths_list_;
	kernel = NavigationPathLayer::createFilter();

}

void NavigationPathLayer::pathCallback(const nav_msgs::Path& path)
{
    boost::recursive_mutex::scoped_lock lock(lock_);

    /* list handling: if new path or update of existing robots path 
        --> if existing delete old costmap manipulation and add new at same position
        --> if new push to list and push id/name of robot to list
    */

    bool isOld = false;
    bool changed = false;
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

    if (!isOld)
    {
        paths_list_.push(path);
    } else
    {
        string oldPath = paths_list_[index_].poses.toString();
        string newPath = path.poses.toString();
        if (oldpath.compare(newpath) != 0)
        {
            changed = true;
        }

        paths_list_[index_] = path; // timestamp is newer
    }

    delete isOld;
    delete index_;
    delete oldPath;
    delete newPath;

    if (changed)
    {
        updateCosts();
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

    for(nav_msgs::Path& path: navigation_path_layers::paths_list_)
    {
        list <vector<int>> positions;
        for(geometry_msgs::PoseStamped pose_ : path.poses)
        {
            vector<int> position = {pose_.pose.position.x, pose_.pose.position.y};
            positions.push(position)
        }
        // add cost hills per path
        costmap = createCostHillChain(positions, costmap);
    }
}

void NavigationPathLayer::setSideInflation(bool inflate)
{
    navigation_path_layers::side_inflation = inflate;
    kernel = navigation_path_layers::createFilter();
}

void NavigationPathLayer::scaleSideInflation(double inflation_scale)
{
    // maximal factor 1 of the costs of the normal path
    navigation_path_layers::inflation_strength = max(min(inflation_scale, 1), 0);
	kernel = navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterSize(int size)
{
    if (size%2 == 1) 
    {
		navigation_path_layers::filter_size = min(size, 9);
    } else
    {
        // minimum size required
        navigation_path_layers::filter_size = min(size-1, 9); // ~ 12.5 cm tolerance with degrading costs
    }
    
    kernel = navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterStrength(int s)
{
    navigation_path_layers::filter_strength = s;
    kernel = navigation_path_layers::createFilter();
}


/* void NavigationPathLayer::resetCosts()
{
    // gesamte Layer auf 0 zurücksetzen
} */

costmap_2d::Costmap2D* NavigationPathLayer::createCostHillChain(list<vector<int>> positions, costmap_2d::Costmap2D costmap) // Pfad übergeben
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

void NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
{
	// sum is for normalization 
	double sum = 0.0;
	int bound = int((filter_size - 1) / 2);
	double[][] kernel = double[filter_size][filter_size];

	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			gauss_r = sqrt(i * i + j * j);
			kernel[i + bound][j + bound] = (exp(-(gauss_r * gauss_r) / gauss_s)) / (M_PI * gauss_s);
			sum += kernel[i + bound][j + bound];
		}
	}

	// normalising the Kernel 
	for (int i = 0; i < filter_size; ++i)
	{
		for (int j = 0; j < filter_size; ++j)
		{
			kernel[i][j] /= sum;
		}
	}

	return kernel;
}

costmap_2d::Costmap2D NavigationPathLayer::useFilter(vector<int> position, costmap_2d::Costmap2D costmap)
{
	int bound = int((filter_size - 1) / 2);
	costmap_2d::Costmap2D _map = costmap; // ToDo prove

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			double current = costmap[position[0] + i][position[1] + j];
			_map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * filter_strength);
		}
	}

	if (side_inflation)
	{/*
		// ToDo:
		vector<int> side = position;
		// orientation einarbeiten
		// add values to "right" side of robot
		side[0] = position[0] + filter_size * x + position[1] + filter_size * y;
		side[1] = position[1] + filter_size * x + position[0] + filter_size * y;

		_map = useSideFilter(side, _map);*/
	}

    return _map;
}

costmap_2d::Costmap2D NavigationPathLayer::useSideFilter(vector<int> position, costmap_2d::Costmap2D costmap)
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