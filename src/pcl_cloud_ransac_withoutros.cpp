#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255,240,245);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud2, 0, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "after rasanc");

  viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color1, "initial cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "after rasanc");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aftercloud(new pcl::PointCloud<pcl::PointXYZ>);
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    reader.read ("model9.pcd", *cloud);
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.85, 1.1);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    <<coefficients->values[1] << " "
    <<coefficients->values[2] << " " 
    <<coefficients->values[3] <<std::endl;
    aftercloud->width  = inliers->indices.size ();
    aftercloud->height = 1;
    aftercloud->points.resize(aftercloud->width * aftercloud->height);
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        // std::cerr << inliers->indices[i]<< "    " <<cloud_filtered->points[inliers->indices[i]].x << " "
        // <<cloud_filtered->points[inliers->indices[i]].y << " "
        // <<cloud_filtered->points[inliers->indices[i]].z << std::endl;
        aftercloud->points[i].x=cloud_filtered->points[inliers->indices[i]].x;
        aftercloud->points[i].y=cloud_filtered->points[inliers->indices[i]].y;
        aftercloud->points[i].z=cloud_filtered->points[inliers->indices[i]].z;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    //利用ExtractIndices根据索引进行点云的提取
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract.filter(*cloud_1);
    pcl::PCDWriter writer;
    writer.write ("after_rasanc.pcd", *cloud_1, false);
    viewer = customColourVis(aftercloud,cloud_filtered);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  return (0);
}