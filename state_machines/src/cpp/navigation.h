#include <iostream>
#include <limits>
// #include <math>
#include <math.h>
#include <queue>

// #include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h> 

#include "orion_actions/NavigationalQuery.h"

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


inline geometry_msgs::Point operator*(const double& mult, const geometry_msgs::Point& p) {
    geometry_msgs::Point output;
    output.x = p.x*mult;
    output.y = p.y*mult;
    output.z = p.z*mult;
    return output;
}
inline geometry_msgs::Point operator+(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    geometry_msgs::Point output;
    output.x = p1.x + p2.x;
    output.y = p1.y + p2.y;
    output.z = p1.z + p2.z;
    return output;
}
inline geometry_msgs::Point operator-(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    geometry_msgs::Point output;
    output.x = p1.x - p2.x;
    output.y = p1.y - p2.y;
    output.z = p1.z - p2.z;
    return output;
}
template <class T> inline double length(const T& vec) {
    /*
    A simple template class for finding the length of a vector.
    */
    return std::sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

template<class T, class U> inline void copyPoint(const T& copying_from, U& copying_to) {
    copying_to.x = copying_from.x; 
    copying_to.y = copying_from.y; 
    copying_to.z = copying_from.z; 
}
template<class T> inline double square(const T& val) { return val*val; }
template<class T, class U> inline double sq_distance_2D(const T& p1, const U& p2) {
    return square(p1.x-p2.x) + square(p1.y-p2.y);
}
template<class T> constexpr geometry_msgs::Point toGeoPoint(const T& p) {
    geometry_msgs::Point output;
    output.x = p.x;
    output.y = p.y;
    output.z = p.z;
    return output;
}


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
        return this->raw_data[i0 + i1*dim0];
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
#define NOT_OCCUPIED 0x00
#define OCCUPIED 0x01
template<IndexType dim0, IndexType dim1> class OccupancyMap : public Array2D<byteImageType, dim0, dim1> {
private:
    double pixel_size;
    double origin_x;
    double origin_y;

public:
    OccupancyMap(const double& pixel_size, const double& origin_x, const double& origin_y) 
        : Array2D<byteImageType, dim0, dim1>(), origin_x(origin_x), origin_y(origin_y) {};
    ~OccupancyMap() {};

public:
    void setAtCoordinate(const double& x, const double&y, const byteImageType& set_to) {
        IndexType i0 = this->spaceToIndex(x, origin_x);
        IndexType i1 = this->spaceToIndex(y, origin_y);
        this->setAtCoordinate(i0, i1, set_to);
    }
    void setAtCoordinate(const IndexType& i0, const IndexType& i1, const byteImageType& set_to) {
        if (i0 >= dim0 || i0 < 0 || i1 >= dim1 || i1 < 0)
            return;
        this->get(i0, i1) = set_to;
    }
    void orAtCoordinate(const IndexType& i0, const IndexType& i1, const byteImageType& or_with) {
        if (i0 >= dim0 || i0 < 0 || i1 >= dim1 || i1 < 0)
            return;
        byteImageType& editing = this->get(i0, i1); 
        editing = editing | or_with;
    }
    byteImageType getAtCoordinate(const double& x, const double&y) {
        IndexType i0 = this->spaceToIndex(x, origin_x);
        IndexType i1 = this->spaceToIndex(y, origin_y);
        return this->getAtCoordinate(i0, i1);
    }
    byteImageType getAtCoordinate(const IndexType& i0, const IndexType& i1) {
        if (i0 >= dim0 || i0 < 0 || i1 >= dim1 || i1 < 0)
            return BEYOND_MAP_BOUNDARY;
        return this->get(i0, i1);
    }


public:
    void setWithinRadius(const double& x, const double& y, const double& radius) {
        for (double i = x-radius; i < x+radius; i+=this->pixel_size) {
            double y_delta = sqrt(radius*radius - i*i);
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

    geometry_msgs::Point findNavGoal(const geometry_msgs::Point& current_location, geometry_msgs::Point& navigating_to, 
        const double& distance_from_target, const double& allowed_error=0.2) {
        
        const byteImageType PIXEL_IN_QUEUE = 0x40;
        const byteImageType PIXEL_VISITED = 0x80;

        geometry_msgs::Point output;

        // current_location.z = 0; navigating_to.z = 0;

        geometry_msgs::Point nav_delta = current_location - navigating_to;
        geometry_msgs::Point starting_query_point = navigating_to + distance_from_target/
            std::sqrt(sq_distance_2D(current_location, navigating_to)) * nav_delta;

        IndexType x_index = spaceToIndex(starting_query_point.x, this->origin_x);
        IndexType y_index = spaceToIndex(starting_query_point.y, this->origin_y);

        std::queue<IndexType> x_index_queue;
        std::queue<IndexType> y_index_queue;
        std::queue<IndexType> x_coord_queue;
        std::queue<IndexType> y_coord_queue;
        

        x_index_queue.push(x_index);
        y_index_queue.push(y_index);
        x_coord_queue.push(starting_query_point.x);
        y_coord_queue.push(starting_query_point.y);

        double x,y;

        const int LEN_DELTAS = 4;
        IndexType deltas[] = {0,1,0,-1}; 

        geometry_msgs::Point trial_point;

        while(x_index_queue.empty() == false) {
            x_index = x_index_queue.front(); x_index_queue.pop();
            y_index = y_index_queue.front(); y_index_queue.pop();
            x = x_coord_queue.front(); x_coord_queue.pop();
            y = y_coord_queue.front(); y_coord_queue.pop();

            if (this->getAtCoordinate(x_index, y_index) & OCCUPIED == 0) {
                output.x = x;
                output.y = y;
                return output;
            }
            else {
                // this->orAtCoordinate(x_index, y_index, PIXEL_VISITED);
                for (char i = 0; i<LEN_DELTAS; i++) {
                    IndexType trial_x_index = x_index + deltas[i];
                    IndexType trial_y_index = y_index + deltas[(i+1)%LEN_DELTAS];

                    trial_point.x = this->indexToSpace(trial_x_index, this->origin_x);
                    trial_point.y = this->indexToSpace(trial_y_index, this->origin_y);

                    if (length(trial_point-navigating_to) < distance_from_target+allowed_error && 
                        length(trial_point-navigating_to) > distance_from_target-allowed_error &&
                        this->getAtCoordinate(trial_x_index, trial_y_index) & PIXEL_IN_QUEUE == 0) {

                        x_index_queue.push(trial_x_index);
                        y_index_queue.push(trial_y_index);
                        x_coord_queue.push(trial_point.x);
                        y_coord_queue.push(trial_point.y);

                        this->orAtCoordinate(trial_x_index, trial_y_index, PIXEL_IN_QUEUE);
                    }
                }
            }
        }

        return output;
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
constexpr double OCCUPANCY_MAP_WIDTH = 3;   //m
constexpr double PIXEL_SIZE = 0.01;         //m
constexpr IndexType OCCUPANCY_MAP_PIXEL_WIDTH = OCCUPANCY_MAP_WIDTH/PIXEL_SIZE;
// When we have a point that has a pixel above it, we need to fill the occupancy map up to a 
// certain radius around. This is the radius around which we fill.
constexpr double FILL_RADIUS = 0.07;        //m
class GettingSuitableNavGoal {
public:
    // The point we want to get close to.
    geometry_msgs::Point location_of_interest;
    geometry_msgs::Point navigate_to;
    geometry_msgs::Point current_location;


    // The distance away from the object we want to end up in mm.
    double distance_away;


    pcl::shared_ptr<PointCloud> shared_cloud;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;


public:
    ros::NodeHandle& node_handle;


public:
    GettingSuitableNavGoal(ros::NodeHandle& node_handle);
    ~GettingSuitableNavGoal();

public:
    /*
    Gets the distance between two points.
    */
    inline double sq_distance(const Point_T& p1, const Point_T& p2);
    inline double distance(const Point_T& p1, const Point_T& p2);


    /*
    Transforms the point cloud to the map frame.
    */
    void transformPointCloud(const PointCloud& original, PointCloud& output);

    /*
    Removes all the points at floor level.
    */
    void filterOutFloor_FarObjs(pcl::PointIndices& output);




public:
    /*
    Creates the occupancy map.
    */
    void createOccupancyMap(
        OccupancyMap<OCCUPANCY_MAP_PIXEL_WIDTH, OCCUPANCY_MAP_PIXEL_WIDTH>& occupancy_map, 
        const pcl::PointIndices& indices);
};


/*
The service callback.

Entrypoint into the entire system.
*/
bool serviceCallback(
    orion_actions::NavigationalQuery::Request& req,
    orion_actions::NavigationalQuery::Response& resp);

/*
The main function.
*/
int main(int argc, char **argv);
