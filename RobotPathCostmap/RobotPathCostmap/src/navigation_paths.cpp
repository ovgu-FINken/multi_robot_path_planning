#include <string>
#include <nav_msgs>
#include <list>
#include <cmath>
#include <RobotPathCostmap/RobotPathCostmap/include/navigation_paths.h>

//http://docs.ros.org/api/nav_msgs/html/msg/Path.html

// ToDo: setFilterStrength()-function

namespace navigation_path_layers 
{

    static double* gauss_a = 1;
    static const double gauss_sigma = 1.0;
	static const double  gauss_r, gauss_s = 2.0 * gauss_sigma * gauss_sigma;

void NavigationPathLayer::onInitialize()
{

    bool* side_inflation = false;
	int* filter_strength = 1; // ToDo: adapt to suitable default value for costs
    int* inflation_size = 1;
    int* filter_size = 20; 
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
    bool changed = false;
    unsigned int* index_ = paths_list_.size(); // this value cannot be assigned in for-loop (as back check)

    // 2 Filter einfügen: einmal mit Seitenbias, einmal ohne bzw. aus launch-File übernehmen, welcher der beiden erstellt werden soll

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



void NavigationPathLayer::updateBounds()
{
    // Grenzen der Costmap neu setzen, um alle benötigten Punkte zu beinhalten
}

void NavigationPathLayer::updateCosts()
{
    // Wenn Änderung bei übergebenen Pfaden (nur dann Funktionsaufruf)
    // Kosten auf 0 zurücksetzen
    resetCosts();

    for(nav_msgs::Path& path: navigation_path_layers::paths_list_)
    {
        std::vector positions<std::vector<int>> positions;
        for(geometry_msgs::PoseStamped pose_ : path.poses)
        {
            std::vector<int> position = {pose_.pose.position.x, pose_.pose.position.y};
            positions.push(position)
        }
        // Kostenberge je Pfad einfügen
        createCostHillChain(positions);
    }
}

void NavigationPathLayer::setSideInflation(bool inflate)
{
    navigation_path_layers::side_inflation* = inflate;
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::scaleSideInflation(float* inflation_scale)
{
    // maximal factor 1 of the costs of the normal path
    navigation_path_layers::inflation_size* = min(inflation_scale, 1);
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterSize(int size)
{
    navigation_path_layers::filter_size* = size;
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::setFilterStrength(float s)
{
    navigation_path_layers::gauss_a = s;
    navigation_path_layers::createFilter();
}

void NavigationPathLayer::inflate_side()
{
// optional; done later
// create costs on other robots' right side to always pass them on the other
}

void NavigationPathLayer::resetCosts()
{
    // gesamte Map auf 0 zurücksetzen
}

void NavigationPathLayer::createCostHillChain(std::vector<std::vector<int>> positions) // Pfad übergeben
{
    // increase costs along the path
	for (unsigned int pos = 0; pos < positions.size(); i++)
	{
		std::vector<int> position = positions[pos];
		useFilter(position);
	}
 
	// Später eventuell
    // entlang des Pfades linear abnehmende/zunehmende Kosten? (je weiter in die "Zukunft" desto weniger /mehr Einfluss auf die Costmap 
    // (Unsicherheit als geringe Wahrscheinlichkeit oder als größeren Puffer betrachten))

    // open-cv gaußfilter costmap konvertieren ggfs
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
}

void NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
{
	// sum is for normalization 
	double sum = 0.0;
	int bound = int((filter_size - 1) / 2);

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
		// add values to "left" side of robot
		// sum += these values
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

void useFilter(std::vector<int> position)
{
	int bound = int((filter_size - 1) / 2);

	// für jeden Pixel in der Konvolution den max-Wert von Filterwert an der Stelle und aktuellem Wert nehmen
	for (unsigned int i = -bound; i <= bound; i++)
	{
		for (unsigned int j = -bound; j <= bound; j++)
		{
			double current = map[position[0] + i][position[1] + j];
			map[position[0] + i][position[1] + j] = max(current, kernel[i][j] * filter_strength);
		}
	}
}

}

PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)