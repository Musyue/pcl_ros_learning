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
int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                        cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;

    reader.read ("3dview_round_3.pcd", *cloud);
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 2.1);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.0001);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    <<coefficients->values[1] << " "
    <<coefficients->values[2] << " " 
    <<coefficients->values[3] <<std::endl;

    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        std::cerr << inliers->indices[i] << "    " <<cloud_filtered->points[inliers->indices[i]].x << " "
        <<cloud_filtered->points[inliers->indices[i]].y << " "
        <<cloud_filtered->points[inliers->indices[i]].z << std::endl;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    //利用ExtractIndices根据索引进行点云的提取
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract.filter(*cloud_1);
    pcl::PCDWriter writer;
    writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_1, false);

  return (0);
}