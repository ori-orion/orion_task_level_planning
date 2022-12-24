#include "navigation.h"


inline double distance(const Point_T& p1, const Point_T& p2) {
    Point_T vec(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);

    return std::sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

void pointCloudCallback(const sensor_msgs::PointCloud2& msg) {

    // mdo::Transformable transform_getter;
    
    ros::Time time_message_received = ros::Time::now(); 
    const std::string target = "map";
    const std::string source = "head_rgbd_sensor_rgb_frame";

    // geometry_msgs::TransformStamped current_transform = 
    //     transform_getter.getTransform(target, source, ros::Time(0));

    // ROS_INFO("Data recieved: is_bigendian=", msg.is_bigendian, " data_size=", msg.data.size());
    std::cout 
        << "Data recieved: is_bigendian=" << msg.is_bigendian 
        << " data_size=" << msg.data.size() 
        << " (row_step, point_step)=(" << msg.row_step << ", " << msg.point_step << ")"
        << " fields.size()=" << msg.fields.size()
        << std::endl;
    for(int i=0; i<msg.fields.size(); i++) {
        std::cout 
            << "\tname=" << msg.fields[i].name
            << " \toffset=" << msg.fields[i].offset
            << std::endl;
    }

    // std::cout << "Pre boost creation" << std::endl;
    // boost::shared_ptr<mdo::PointCloud> cloud;
    // std::cout << "Pre fromROSMsg function call" << std::endl;
    // pcl::fromROSMsg(msg, *cloud);

    std::cout << "Creating shared_cloud" << std::endl;
    pcl::shared_ptr<PointCloud> shared_cloud(new PointCloud());

    std::cout << "Loading point cloud." << std::endl;
    PointCloud& cloud = *(shared_cloud.get());
    pcl::fromROSMsg(msg, cloud);

    std::cout << "Received cloud with " << cloud.points.size() << " points.";
    
    // pcl::io::savePLYFileBinary("./point_cloud", cloud);

    // std::cout << current_transform << std::endl;

    //... populate cloud
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(shared_cloud);
    while (!viewer.wasStopped ())
    {
    }

    ros::shutdown();
}

int main(int argc, char **argv) {
    // ROS_DEBUG("Hello world");
    std::cout << "Hello world" << std::endl;

    // ros::init(argc, argv, "matthewm_vision_node");
    // ros::NodeHandle n;

    // std::cout << "Waiting 2 secs" << std::endl;
    // mdo::Transformable transform_getter;
    // ros::Time::sleepUntil(ros::Time::now() + ros::Duration(2));
    // std::cout << "Finished waiting" << std::endl;

    // ros::Subscriber sub = n.subscribe("/hsrb/head_rgbd_sensor/depth_registered/points", 2, chatterCallback);

    // ros::spin();

    return 0;
}