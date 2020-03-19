#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_kdtree_test");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub_kdtree = nh.advertise<sensor_msgs::PointCloud2> ("pcl_kdtree_rand_output", 1);
    ros::Publisher pcl_pub_kdtree_k_search = nh.advertise<sensor_msgs::PointCloud2> ("pcl_kdtree_k_search_output", 1);
    ros::Publisher pcl_pub_kdtree_r_search = nh.advertise<sensor_msgs::PointCloud2> ("pcl_kdtree_r_search_output", 1);
    ros::Publisher pcl_pub_kdtreesearch_point = nh.advertise<sensor_msgs::PointCloud2> ("pcl_kdtree_search_point", 1);

    sensor_msgs::PointCloud2 kdtree_rand_output;
    sensor_msgs::PointCloud2 kdtree_k_search_output;
    sensor_msgs::PointCloud2 kdtree_r_search_output;
    sensor_msgs::PointCloud2 kdtree_search_point;

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud->width=1000;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0;i<cloud->points.size();++i)
    {
        cloud->points[i].x=1024*rand()/(RAND_MAX + 1.0f);
        cloud->points[i].y=1024*rand()/(RAND_MAX + 1.0f);
        cloud->points[i].z=1024*rand()/(RAND_MAX + 1.0f);
    }
    pcl::toROSMsg(*cloud, kdtree_rand_output);
    kdtree_rand_output.header.frame_id = "odom";

    pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x=1024*rand()/(RAND_MAX + 1.0f);
    searchPoint.y=1024*rand()/(RAND_MAX + 1.0f);
    searchPoint.z=1024*rand()/(RAND_MAX + 1.0f);
    pcl::PointCloud<pcl::PointXYZ> searchcloud;
    searchcloud.width  = 1;
    searchcloud.height = 1;
    searchcloud.points.resize(searchcloud.width * searchcloud.height);

    for (size_t i = 0; i < searchcloud.points.size (); ++i)
    {
        searchcloud.points[i].x = searchPoint.x;
        searchcloud.points[i].y = searchPoint.y;
        searchcloud.points[i].z = searchPoint.z;
    }
    pcl::toROSMsg(searchcloud, kdtree_search_point);
    kdtree_search_point.header.frame_id = "odom";

    int K=10;
    std::vector<int>pointIdxNKNSearch(K);
    std::vector<float>pointNKNSquaredDistance(K);
    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with K="<< K <<std::endl;
    pcl::PointCloud<pcl::PointXYZ> k_searchcloud;

    if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
    {
        k_searchcloud.width  = pointIdxNKNSearch.size();
        k_searchcloud.height = 1;
        k_searchcloud.points.resize(k_searchcloud.width * k_searchcloud.height);
        for(size_t i=0;i<pointIdxNKNSearch.size();++i)
        {
            std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
            <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;

            // display result
            k_searchcloud.points[i].x = cloud->points[ pointIdxNKNSearch[i] ].x ;
            k_searchcloud.points[i].y = cloud->points[ pointIdxNKNSearch[i] ].y;
            k_searchcloud.points[i].z = cloud->points[ pointIdxNKNSearch[i] ].z;
        }
    }
    pcl::toROSMsg(k_searchcloud, kdtree_k_search_output);
    kdtree_k_search_output.header.frame_id = "odom";
    // radius r search method
    std::vector<int>pointIdxRadiusSearch;
    std::vector<float>pointRadiusSquareDistance;
    float radius=256.0f*rand()/(RAND_MAX+1.0f);
    std::cout<<"Neighbors within radius search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with radius="<< radius <<std::endl;
    pcl::PointCloud<pcl::PointXYZ> r_searchcloud;
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquareDistance) >0 )
    {
        r_searchcloud.width  = pointIdxRadiusSearch.size();
        r_searchcloud.height = 1;
        r_searchcloud.points.resize(r_searchcloud.width * r_searchcloud.height);
        for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
        {
            std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
            <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
            <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
            <<" (squared distance: "<<pointRadiusSquareDistance[i] <<")"<<std::endl;
            r_searchcloud.points[i].x = cloud->points[ pointIdxRadiusSearch[i] ].x ;
            r_searchcloud.points[i].y = cloud->points[ pointIdxRadiusSearch[i] ].y;
            r_searchcloud.points[i].z = cloud->points[ pointIdxRadiusSearch[i] ].z;
        }
    }
    pcl::toROSMsg(r_searchcloud, kdtree_r_search_output);
    kdtree_r_search_output.header.frame_id = "odom";
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub_kdtree.publish(kdtree_rand_output);
        pcl_pub_kdtreesearch_point.publish(kdtree_search_point);
        pcl_pub_kdtree_k_search.publish(kdtree_k_search_output);
        pcl_pub_kdtree_r_search.publish(kdtree_r_search_output);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

