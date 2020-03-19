#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1000);

    sensor_msgs::PointCloud2 output;

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::io::loadPCDFile ("3dview_round_3.pcd", cloud);
    for(size_t i=0;i<cloud.points.size();++i)
    {
        std::cout<< "num:"<<i<<"  "<< cloud.points[i].x<<"  "<<cloud.points[i].y<<" "<<cloud.points[i].z<<"  "<< cloud.points[i].r<<"  "<<cloud.points[i].g<<" "<<cloud.points[i].b<<std::endl;
    }
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

