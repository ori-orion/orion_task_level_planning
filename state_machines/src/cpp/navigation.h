#include <iostream>
#include <limits>
#include <math>

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


/* Base class for all 2D arrays.
 * 
 * VERY simple. */
#define IndexType int
template<class T, IndexType dim0, IndexType dim1> class Array2D {
private:
    T raw_data[dim1*dim0];

public:
    Array2D() : raw_data() {};
    ~Array2D() {};

public:
    T& get(const IndexType& i0, const IndexType& i1) {
        return this->raw_data[i0 + i1*dim0]
    }
};

/* An image class.
 * 
 * We need to encode an occupancy map somehow. (A set of points the robot cannot travel to.) 
 * This will do that. 
 * The bottom left of each dimension will be encoded by (origin_x, origin_y). 
 */
#define byteImageType unsigned char
#define BEYOND_MAP_BOUNDARY UCHAR_MAX
#define NOT_OCCUPIED 0
#define OCCUPIED 1
template<IndexType dim0, IndexType dim1> class OccupancyMap : public Array2D<byteImageType, dim0, dim1> {
private:
    double pixel_size;
    double origin_x;
    double origin_y;

public:
    OccupancyMap(const double& pixel_size, const double& origin_x, const double& origin_y) 
        : Array2D<char, dim0, dim1>(), origin_x(origin_x), origin_y(origin_y) {};
    ~OccupancyMap() {};

public:
    void setAtCoordinate(const double& x, const double&y, const byteImageType& set_to) {
        IndexType i0 = this->spaceToIndex(x, origin_x);
        IndexType i1 = this->spaceToIndex(y, origin_y);
        if (i0 >= dim0 || i0 < 0 || i1 >= dim1 || i1 < 0)
            return;
        
        this->get(i0, i1) = set_to;
    }
    byteImageType getAtCoordinate(const double& x, const double&y) {
        IndexType i0 = this->spaceToIndex(x, origin_x);
        IndexType i1 = this->spaceToIndex(y, origin_y);
        if (i0 >= dim0 || i0 < 0 || i1 >= dim1 || i1 < 0)
            return BEYOND_MAP_BOUNDARY;

        return this->get(i0, i1);
    }

public:
    void setWithinRadius(const double& x, const double& y, const double& radius) {
        for (double i = x-radius; i < x+radius; i+=this->pixel_size) {
            double y_delta = sqrt(radius*radius - i*i)
            for (double j = y-y_delta; j < y+y_delta; j+=this->pixel_size) {
                this->setAtCoordinate(i, j, OCCUPIED);
            }
        }
    }

    void print() {
        const unsigned short cells_per_print = 5;

        for (int i = 0; i < dim0; i +=cells_per_print) {
            for (int j = 0; j < dim1; j += cells_per_print) {
                
                bool print_space = true;
                for (int k = 0; k < cells_per_print; k++) {
                    for (int l = 0; l < cells_per_print; l++) {
                        if (this->get(i+k, j+l) == OCCUPIED)
                            print_space = false;
                    }
                }
                if (print_space == true) std::cout << " ";
                else std::cout << "X";

            }
            std::cout << std::endl;
        }
    }

private:
    IndexType spaceToIndex(const double& coordinate, const double& starting_coord) {
        return (IndexType)((coordinate-starting_coord)/pixel_size);
    }
    double indexToSpace(const IndexType& index, const double& starting_coord) {
        return pixel_size * index + starting_coord;
    }
};


/*
There are multiple situations where we have something we might want
to pick up, or put down in a certain location. This involves 
navigating to a place NEARBY BUT NOT ONTOP OF a certain goal 
location. This class will work out the suitable navigation goal 
involved.
*/
const double OCCUPANCY_MAP_WIDTH = 3;   //m
const double PIXEL_SIZE = 0.01;         //m
const IndexType OCCUPANCY_MAP_WIDTH = OCCUPANCY_MAP_WIDTH/PIXEL_SIZE;
// When we have a point that has a pixel above it, we need to fill the occupancy map up to a 
// certain radius around. This is the radius around which we fill.
const double FILL_RADIUS = 0.07;        //m
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

private:
    /*
    Creates the occupancy map.
    */
    void createOccupancyMap(
        OccupancyMap<OCCUPANCY_MAP_WIDTH, OCCUPANCY_MAP_WIDTH>& occupancy_map, 
        const pcl::PointIndices& indices);
};


/*
The main function.
*/
int main(int argc, char **argv);
