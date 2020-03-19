// #include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
main (int argc, char **argv)
{
    // ros::init (argc, argv, "pcl_write_test");

    // ros::NodeHandle nh;
    srand((unsigned int)time(NULL));

    float resolution =32.0f;//octree voxel size
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree(resolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    cloudA->width=128;
    cloudA->height=1;
    cloudA->points.resize(cloudA->width*cloudA->height);
    for (size_t i=0; i<cloudA->points.size (); ++i)
    {
        cloudA->points[i].x =64.0f* rand () / (RAND_MAX +1.0f);
        cloudA->points[i].y =64.0f* rand () / (RAND_MAX +1.0f);
        cloudA->points[i].z =64.0f* rand () / (RAND_MAX +1.0f);
    }
    for (size_t i=0; i<cloudA->points.size (); ++i)
    {
        std::cout<<"#:"<<i<<"CloudA"<<cloudA->points[i].x<<" "<<cloudA->points[i].y<<" "<<
        cloudA->points[i].z<<std::endl;
    }
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    // ros::spin();
    octree.switchBuffers();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
    cloudB->width=128;
    cloudB->height=1;
    cloudB->points.resize(cloudB->width*cloudB->height);
    for (size_t i=0; i<cloudB->points.size (); ++i)
    {
        cloudB->points[i].x =64.0f* rand () / (RAND_MAX +1.0f);
        cloudB->points[i].y =64.0f* rand () / (RAND_MAX +1.0f);
        cloudB->points[i].z =64.0f* rand () / (RAND_MAX +1.0f);
    }
    for (size_t i=0; i<cloudB->points.size (); ++i)
    {
        std::cout<<"#:"<<i<<"CloudB: "<<cloudB->points[i].x<<" "<<cloudB->points[i].y<<" "<<
        cloudB->points[i].z<<std::endl;
    }
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();
    std::vector<int>newPointIdxVector;
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);
    std::cout<<"Output from getPointIndicesFromNewVoxels:"<<std::endl;
    for(size_t i=0;i<newPointIdxVector.size();++i)
    {
        std::cout<<i<<"#Index:"<<newPointIdxVector[i]<<" Point:"
        <<cloudB->points[newPointIdxVector[i]].x <<" "
        <<cloudB->points[newPointIdxVector[i]].y <<" "
        <<cloudB->points[newPointIdxVector[i]].z <<std::endl;
    }
    return 0;
}

