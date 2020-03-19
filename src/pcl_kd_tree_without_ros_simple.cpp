// #include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>

main (int argc, char **argv)
{
    // ros::init (argc, argv, "pcl_write_test");

    // ros::NodeHandle nh;
    srand(time(NULL));
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
    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.y=1024.0f*rand()/(RAND_MAX + 1.0f);
    searchPoint.z=1024.0f*rand()/(RAND_MAX + 1.0f);
    int K=10;
    std::vector<int>pointIdxNKNSearch(K);
    std::vector<float>pointNKNSquaredDistance(K);
    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with K="<< K <<std::endl;
    if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
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
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) >0 )
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

