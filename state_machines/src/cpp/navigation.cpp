#include "navigation.h"

GettingSuitableNavGoal::GettingSuitableNavGoal(ros::NodeHandle& node_handle) 
    : location_of_interest(), distance_away(), shared_cloud(new PointCloud()), 
    tf_buffer(), tf_listener(tf_buffer), node_handle(node_handle) {};
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


static ros::NodeHandle* node_handle;


bool serviceCallback(orion_actions::NavigationalQuery::Request& req, orion_actions::NavigationalQuery::Response& resp) {

    boost::shared_ptr<const sensor_msgs::PointCloud2> msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "/hsrb/head_rgbd_sensor/depth_registered/points",
        ros::Duration(1));

    GettingSuitableNavGoal getting_suitable_nav_goal(*node_handle);

    ros::Time time_message_received = ros::Time::now(); 
    const std::string target = "map";
    const std::string source = "head_rgbd_sensor_rgb_frame";

    getting_suitable_nav_goal.location_of_interest = req.navigating_within_reach_of;  
    getting_suitable_nav_goal.current_location = req.current_pose;
    getting_suitable_nav_goal.distance_away = req.distance_from_obj;

    PointCloud cloud;
    pcl::fromROSMsg(*msg, cloud);
    getting_suitable_nav_goal.transformPointCloud(cloud, *(getting_suitable_nav_goal.shared_cloud.get()));

    std::cout << "Received cloud with " << getting_suitable_nav_goal.shared_cloud->points.size() << " points.";
    
    pcl::PointIndices indices;
    getting_suitable_nav_goal.filterOutFloor_FarObjs(indices);

    OccupancyMap<OCCUPANCY_MAP_PIXEL_WIDTH, OCCUPANCY_MAP_PIXEL_WIDTH> occupancy_map(
        PIXEL_SIZE, 
        getting_suitable_nav_goal.location_of_interest.x-OCCUPANCY_MAP_WIDTH/2,
        getting_suitable_nav_goal.location_of_interest.x-OCCUPANCY_MAP_WIDTH/2);
    getting_suitable_nav_goal.createOccupancyMap(occupancy_map, indices);

    occupancy_map.findNavGoal(
        getting_suitable_nav_goal.current_location, 
        getting_suitable_nav_goal.location_of_interest, 
        getting_suitable_nav_goal.distance_away, 
        0.2);
    
    // pcl::io::savePLYFileBinary("./point_cloud", cloud);

    // std::cout << current_transform << std::endl;

    //... populate cloud
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud(this->shared_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

    geometry_msgs::Point nav_delta = getting_suitable_nav_goal.current_location - getting_suitable_nav_goal.navigate_to;
    nav_delta = (1/length(nav_delta)) * nav_delta;

    resp.navigate_to.position = getting_suitable_nav_goal.navigate_to;
    resp.navigate_to.orientation.w = nav_delta.x;
    resp.navigate_to.orientation.z = nav_delta.y;

    return true;
}


int main(int argc, char **argv) {
    // ROS_DEBUG("Hello world");
    std::cout << "Hello world" << std::endl;



    ros::init(argc, argv, "tlp/nav_goal_getter");
    ros::NodeHandle n;
    node_handle = &n;

    ros::ServiceServer service = n.advertiseService(
        "tlp/get_nav_goal",
        serviceCallback);
    // std::cout << "Waiting 2 secs" << std::endl;
    // Transformable transform_getter;
    // ros::Time::sleepUntil(ros::Time::now() + ros::Duration(2));
    // std::cout << "Finished waiting" << std::endl;

    // ros::Subscriber sub = n.subscribe("/hsrb/head_rgbd_sensor/depth_registered/points", 2, chatterCallback);

    ros::spin();

    return 0;
}
