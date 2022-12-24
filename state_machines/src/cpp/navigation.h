#include <iostream>

// #include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h> 

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>


// #include <math>

using Point_T = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point_T>;



/*
There are multiple situations where we have something we might want
to pick up, or put down in a certain location. This involves 
navigating to a place NEARBY BUT NOT ONTOP OF a certain goal 
location. This class will work out the suitable navigation goal 
involved.
*/
class GettingSuitableNavGoal {
private:
    // The point we want to get close to.
    Point_T location_of_interest;

    // The distance away from the object we want to end up in mm.
    double distance_away;

    
    // The search tree for finding the closest points.
    pcl::search::Search<Point_T>::Ptr search_tree;

    pcl::shared_ptr<PointCloud> shared_cloud;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;


public:
    GettingSuitableNavGoal(
        const Point_T& location_of_interest,
        const double& distance_away=500);
    ~GettingSuitableNavGoal();

private:
    /*
    Gets the distance between two points.
    */
    inline double sq_distance(const Point_T& p1, const Point_T& p2);
    inline double distance(const Point_T& p1, const Point_T& p2);

    /*
    Returns the squared distance between two points, but does so only in the plane. 

    For navigation, we're only really interested in that bit anyway.
    */
    inline double sq_distance_2D(const Point_T& p1, const Point_T& p2);


    /*
    Transforms the point cloud to the map frame.
    */
    void transformPointCloud(const PointCloud& original, PointCloud& output);

    /*
    Removes all the points at floor level.
    */
    void filterOutFloor_FarObjs(pcl::PointIndices& output);

    
    /*
    The callback for getting a point cloud as input.

    Is the callback for a message of type point cloud.
    */
    void pointCloudCallback(const sensor_msgs::PointCloud2& msg);

    
private:
    /*
    This function creates the search tree if necessary. It then returns the search tree.
    */
    pcl::search::Search<Point_T>::Ptr getSearchTree();
};


/*
The main function.
*/
int main(int argc, char **argv);
