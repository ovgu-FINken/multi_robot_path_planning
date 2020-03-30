#include <string>
#include <nav_msgs>
#include <list>
#include <cmath>
#include "./../include/RobotPathCostmap/navigation_paths.h">

// ToDo: Libs einbinden
// ToDo: launch-file
// ToDo: Package.xml

// http://docs.ros.org/api/nav_msgs/html/msg/Path.html


namespace navigation_path_layers 
{

    static double* gauss_a = 1;
    static const double gauss_sigma = 1.0;
	static const double  gauss_r, gauss_s = 2.0 * gauss_sigma * gauss_sigma;

void NavigationPathLayer::onInitialize()
{

    bool* side_inflation = false;
	int* filter_strength = 1; 
    int* inflation_size = 1;
    int* filter_size = 9; // resolution: 0.050000 meters / pixel ; 4 pixel needed for robot itself
    first_time_ = true;
    ros::NodeHandle nh("~/" + name_), g_nh; // ToDo: check cooperation with other Multi-robot-path-planning-files
    paths_sub_ = nh.subscribe("/paths", 1, &NavigationPathLayer::pathCallback, this);
    list<Path> paths_list_;
    NavigationPathLayer::createFilter();

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
        std::string oldPath = paths_list_[index_].poses.toString();
        std::string newPath = path.poses.toString();
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
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}

void NavigationPathLayer::updateCosts()
{
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    // function call only if given path changed
    // reset costs to 0
    resetCosts();

    for(nav_msgs::Path& path: navigation_path_layers::paths_list_)
    {
        std::vector positions<std::vector<int>> positions;
        for(geometry_msgs::PoseStamped pose_ : path.poses)
        {
            std::vector<int> position = {pose_.pose.position.x, pose_.pose.position.y};
            positions.push(position)
        }
        // add cost hills per path
        costmap = createCostHillChain(positions, costmap);
    }
}

void NavigationPathLayer::setSideInflation(bool* inflate)
{
    navigation_path_layers::side_inflation = *inflate;
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::scaleSideInflation(double* inflation_scale)
{
    // maximal factor 1 of the costs of the normal path
    navigation_path_layers::inflation_size = std::min(*inflation_scale, 1);
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterSize(int* size)
{
    if (size%2 == 1) 
    {
    navigation_path_layers::filter_size = *size;
    } else
    {
        // minimum size required
        navigation_path_layers::filter_size = std::max(*size-1, 9); // ~ 12.5 cm tolerance with degrading costs
    }
    
    // navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterStrength(int* s)
{
    navigation_path_layers::filter_strength = *s;
    // navigation_path_layers::createFilter();
}

void NavigationPathLayer::inflate_side()
{
	// optional; done later
	// create costs on other robots' right side to always pass them on the other
}

void NavigationPathLayer::resetCosts()
{
    // gesamte Layer auf 0 zurücksetzen
}

costmap_2d::Costmap2D* NavigationPathLayer::createCostHillChain(std::vector<std::vector<int>> positions, costmap_2d::Costmap2D* costmap) // Pfad übergeben
{
	costmap_2d::Costmap2D* costmap_ = costmap;
    // increase costs along the path
	for (unsigned int pos = 0; pos < positions.size(); i++)
	{
		std::vector<int> position = positions[pos];
		costmap_ = useFilter(position*, costmap_);
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
	int bound = int((*filter_size - 1) / 2);
	double[][] * kernel = (double*)malloc(*filter_size * sizeof(double));

	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			gauss_r = sqrt(i * i + j * j);
			kernel[i + bound][j + bound] = (exp(-(gauss_r * gauss_r) / gauss_s)) / (M_PI * gauss_s);
			sum += kernel[i + bound][j + bound];
		}
	}

	if (*side_inflation)
	{
        // ToDo: add later
		// add values to "right" side of robot
		// sum += these values
	}

	// normalising the Kernel 
	for (int i = 0; i < *filter_size; ++i)
	{
		for (int j = 0; j < *filter_size; ++j)
		{
			kernel[i][j] /= sum;
		}
	}

	return kernel;
}

costmap_2d::Costmap2D* useFilter(std::vector<int>* position, costmap_2d::Costmap2D* costmap)
{
	int bound = int((filter_size - 1) / 2);
	costmap_2d::Costmap2D _map = *costmap; // ToDo prove

	// for each pixel in the convolution take maximum value of current and calculated value of convolution at this pixel
	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			double current = costmap[*position[0] + i][*position[1] + j];
			_map[*position[0] + i][*position[1] + j] = std::max(current, kernel[i][j] * filter_strength);
		}
	}

    return _map*;
}

}

PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)