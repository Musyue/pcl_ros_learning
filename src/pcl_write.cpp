#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(input, cloud);
    pcl::io::savePCDFileASCII ("write_pcd_test.pcd", cloud);
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe("pcl_output", 10, cloudCB);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width=5;
    cloud.height=1;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width*cloud.height);
    for(size_t i=0;i<cloud.points.size();++i)
    {
        cloud.points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].y=1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].z=1024*rand()/(RAND_MAX+1.0f);
    }
    pcl::io::savePCDFileASCII("test_pcd_write.pcd",cloud);
    std::cerr<<"saved"<<cloud.points.size()<<"data points to test_pcd.pcd"<<std::endl;
    for(size_t i=0;i<cloud.points.size();++i)
    {
        std::cerr<<"  "<<cloud.points[i].x<<"  "<<cloud.points[i].y<<"  "<<cloud.points[i].z<<std::endl;
    }
    ros::spin();

    return 0;
}

