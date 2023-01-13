#include "navigation.h"

static ros::Subscriber pointcloud_subscriber;
static GettingSuitableNavGoal* nav_goal_getter;
static void pointCloudCallback(const sensor_msgs::PointCloud2& msg) {
    
    ros::Time time_message_received = ros::Time::now(); 
    const std::string target = "map";
    const std::string source = "head_rgbd_sensor_rgb_frame";

    // geometry_msgs::TransformStamped current_transform = 
    //     transform_getter.getTransform(target, source, ros::Time(0));

    // ROS_INFO("Data recieved: is_bigendian=", msg.is_bigendian, " data_size=", msg.data.size());
    // std::cout 
    //     << "Data recieved: is_bigendian=" << msg.is_bigendian 
    //     << " data_size=" << msg.data.size() 
    //     << " (row_step, point_step)=(" << msg.row_step << ", " << msg.point_step << ")"
    //     << " fields.size()=" << msg.fields.size()
    //     << std::endl;
    // for(int i=0; i<msg.fields.size(); i++) {
    //     std::cout 
    //         << "\tname=" << msg.fields[i].name
    //         << " \toffset=" << msg.fields[i].offset
    //         << std::endl;
    // }

    PointCloud cloud;
    pcl::fromROSMsg(msg, cloud);
    nav_goal_getter->transformPointCloud(cloud, *(nav_goal_getter->shared_cloud.get()));

    std::cout << "Received cloud with " << nav_goal_getter->shared_cloud->points.size() << " points.";
    
    pcl::PointIndices indices;
    nav_goal_getter->filterOutFloor_FarObjs(indices);

    OccupancyMap<OCCUPANCY_MAP_PIXEL_WIDTH, OCCUPANCY_MAP_PIXEL_WIDTH> occupancy_map(
        PIXEL_SIZE, 
        nav_goal_getter->location_of_interest.x-OCCUPANCY_MAP_WIDTH/2,
        nav_goal_getter->location_of_interest.x-OCCUPANCY_MAP_WIDTH/2);
    nav_goal_getter->createOccupancyMap(occupancy_map, indices);

    occupancy_map.findNavGoal(
        nav_goal_getter->current_location, 
        nav_goal_getter->location_of_interest, 
        nav_goal_getter->distance_away, 
        0.2);
    
    // pcl::io::savePLYFileBinary("./point_cloud", cloud);

    // std::cout << current_transform << std::endl;

    //... populate cloud
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud(this->shared_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

    nav_goal_getter->response_filled_out = true;

    pointcloud_subscriber.shutdown();
}

GettingSuitableNavGoal::GettingSuitableNavGoal(
    const geometry_msgs::Point& location_of_interest, ros::NodeHandle& node_handle, const double& distance_away) 
    : location_of_interest(location_of_interest), distance_away(distance_away),
    shared_cloud(new PointCloud()), tf_buffer(), tf_listener(tf_buffer), node_handle(node_handle) {};
GettingSuitableNavGoal::~GettingSuitableNavGoal() {};
inline double GettingSuitableNavGoal::sq_distance(const Point_T& p1, const Point_T& p2) {
    Point_T vec(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);

    return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}
inline double GettingSuitableNavGoal::distance(const Point_T& p1, const Point_T& p2) {
    return std::sqrt(this->distance(p1, p2));
}
void GettingSuitableNavGoal::transformPointCloud(
    const PointCloud& original, PointCloud& output) {


    ros::Time time_message_received = ros::Time::now(); 
    const std::string target_frame = "map";
    const std::string source_frame = "head_rgbd_sensor_rgb_frame";

    geometry_msgs::TransformStamped transform = this->tf_buffer.lookupTransform(
        target_frame, source_frame, ros::Time(0));
    Eigen::Isometry3d matrix_transform = tf2::transformToEigen(transform);
    Eigen::Affine3d affine_transformation(matrix_transform);

    pcl::transformPointCloud(original, output, affine_transformation);

}
void GettingSuitableNavGoal::filterOutFloor_FarObjs(pcl::PointIndices& output) {
    // All vals are in mm
    const double max_floor_height = 10;
    const double max_height_of_interest = 2000;
    const double max_radius_of_interest = 1000;
    const double max_radius_of_interest_sq = max_radius_of_interest*max_radius_of_interest; 

    for (int i = 0; i < this->shared_cloud->points.size(); i++) {
        if (this->shared_cloud->points[i].z < max_floor_height)
            continue;
        if (this->shared_cloud->points[i].z > max_height_of_interest)
            continue;
        if (sq_distance_2D(this->shared_cloud->points[i], this->location_of_interest) > max_radius_of_interest_sq)
            continue;

        output.indices.push_back(i);
    }
}
void GettingSuitableNavGoal::serviceCallback(orion_actions::NavigationalQuery::Request& req, orion_actions::NavigationalQuery::Response& resp) {
    pointcloud_subscriber = this->node_handle.subscribe("/hsrb/head_rgbd_sensor/depth_registered/points", 1, pointCloudCallback); 

    this->location_of_interest = req.navigating_within_reach_of;  
    this->current_location = req.current_pose;
    this->response_filled_out = false;
    this->distance_away = req.distance_from_obj;

    ros::Duration sleep_duration(0,100000000);
    while (true) {
        if (this->response_filled_out)
            break;

        sleep_duration.sleep();
    }

    resp.navigate_to = this->navigate_to;

    return;
}
pcl::search::Search<Point_T>::Ptr GettingSuitableNavGoal::getSearchTree(){
    if (this->search_tree.get() == nullptr) {
        this->search_tree = boost::shared_ptr<pcl::search::KdTree<Point_T>>(
            new pcl::search::KdTree<Point_T>());
        
        this->search_tree->setInputCloud(this->shared_cloud);
    }
    return this->search_tree;
}
void GettingSuitableNavGoal::createOccupancyMap(
    OccupancyMap<OCCUPANCY_MAP_PIXEL_WIDTH, OCCUPANCY_MAP_PIXEL_WIDTH>& occupancy_map, 
    const pcl::PointIndices& indices) {

    for (auto ptr=indices.indices.begin(); ptr < indices.indices.end(); ptr++) {
        occupancy_map.setWithinRadius(
            this->shared_cloud->points[*ptr].x,
            this->shared_cloud->points[*ptr].y,
            FILL_RADIUS);
    }
}


int main(int argc, char **argv) {
    // ROS_DEBUG("Hello world");
    std::cout << "Hello world" << std::endl;

    // ros::init(argc, argv, "matthewm_vision_node");
    // ros::NodeHandle n;

    // std::cout << "Waiting 2 secs" << std::endl;
    // Transformable transform_getter;
    // ros::Time::sleepUntil(ros::Time::now() + ros::Duration(2));
    // std::cout << "Finished waiting" << std::endl;

    // ros::Subscriber sub = n.subscribe("/hsrb/head_rgbd_sensor/depth_registered/points", 2, chatterCallback);

    // ros::spin();

    return 0;
}
