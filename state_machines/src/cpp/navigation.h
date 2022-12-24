#include <iostream>

// #include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

// #include <pcl/conversions.h>

#include <memory>

#include <math>

using Point_T = pcl::PointXYZRGB;
using PointCloud = pcl::PointCloud<Point_T>;


/*
Gets the distance between two points.
*/
inline double distance(const Point_T& p1, const Point_T& p2);

/*
The callback for getting a point cloud as input.

Is the callback for a message of type point cloud.
*/
void pointCloudCallback(const sensor_msgs::PointCloud2& msg);


/*
The main function.
*/
int main(int argc, char **argv);