#include <string>
#include <nav_msgs>
#include <list>
#include <RobotPathCostmap/RobotPathCostmap/include/navigation_paths.h>

//http://docs.ros.org/api/nav_msgs/html/msg/Path.html


namespace navigation_path_layers 
{

    static float* gauss_a = 1;
    static const int gauss_b = 0;
    static const float gauss_c = 1;

void NavigationPathLayer::onInitialize()
{

    bool* side_inflation = false;
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
        createCostHillChain();
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

void NavigationPathLayer::createCostHillChain(std::vector positions<std::vector<int>> positions) // Pfad übergeben
{
    // Entlang des Pfades Kosten erhöhen ()
    // Gauß-Filter entlang des Pfades
        // wenn side_inflation
            // Gaußfilter mit Seitenbias nutzen
        // sonst
            // normaler Gaußfilter verwenden
    // für jeden Pixel in der Konvolution den max-Wert von Filterwert an der Stelle und aktuellem Wert nehmen

    // entlang des Pfades linear abnehmende/zunehmende Kosten? (je weiter in die "Zukunft" desto weniger /mehr Einfluss auf die Costmap 
    // (Unsicherheit als geringe Wahrscheinlichkeit oder als größeren Puffer betrachten))

    // open-cv gaußfilter costmap konvertieren ggfs
    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
}

void NavigationPathLayer::createFilter() // Größe und side_inflation nutzen
{

}

}

PLUGINLIB_EXPORT_CLASS(navigation_path_layers::NavigationPathLayer, costmap_2d::Layer)