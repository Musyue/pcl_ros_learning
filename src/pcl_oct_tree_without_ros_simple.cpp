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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width=1000;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0;i<cloud->points.size();++i)
    {
        cloud->points[i].x=1024.0f*rand()/(RAND_MAX + 1.0f);
        cloud->points[i].y=1024.0f*rand()/(RAND_MAX + 1.0f);
        cloud->points[i].z=1024.0f*rand()/(RAND_MAX + 1.0f);
    }
    //resolution
    float resolution=128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.y=1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.z=1024.0f*rand()/(RAND_MAX + 1.0f);
    std::vector<int>pointIdxVec;
    if(octree.voxelSearch(searchPoint,pointIdxVec))
    {
        std::cout<<"Neighbors within voxel search at ("<<searchPoint.x
        <<" "<<searchPoint.y
        <<" "<<searchPoint.z<<")"
        <<std::endl;
        for (size_t i=0; i<pointIdxVec.size (); ++i)
            std::cout<<"    "<< cloud->points[pointIdxVec[i]].x 
            <<" "<< cloud->points[pointIdxVec[i]].y 
            <<" "<< cloud->points[pointIdxVec[i]].z <<" "<<pointIdxVec[i]<<std::endl;
    }
    int K=10;
    std::vector<int>pointIdxNKNSearch(K);
    std::vector<float>pointNKNSquaredDistance(K);
    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with K="<< K <<std::endl;
    if(octree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
    {
        for(size_t i=0;i<pointIdxNKNSearch.size();++i)
        {
            std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
            <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;
        }
    }
    // radius r search method
    std::vector<int>pointIdxRadiusSearch;
    std::vector<float>pointRadiusSquareDistance;
    float radius=256.0f*rand()/(RAND_MAX+1.0f);
    std::cout<<"Neighbors within radius search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with radius="<< radius <<std::endl;
    if ( octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) >0 )
    {
        for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
            std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
            <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
            <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
            <<" (squared distance: "<<pointRadiusSquareDistance[i] <<")"<<std::endl;
    }
    // ros::spin();

    return 0;
}

