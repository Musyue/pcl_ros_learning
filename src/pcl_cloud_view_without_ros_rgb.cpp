// #include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <vector>
#include <ctime>
#include <pcl/console/parse.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointType;
int user_data;
void printUsage(const char* progName)
{
      std::cout << "\n\nUsage: "<<progName<<" [options] <xxx.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "\n\n";
}
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0,0.5,1.0);
    pcl::PointXYZ o;
    o.x=1.0;
    o.y=0;
    o.z=0;
    viewer.addSphere(o,0.25,"sphere",0);
    std::cout<<"I only run one time"<<std::endl;
}
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0,0.5,1.0);
    static unsigned count=0;
    std::stringstream ss;
    ss<<"Once per viewer loop:"<<count++;
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);
    user_data++;
}
main (int argc, char **argv)
{
    // ros::init (argc, argv, "pcl_write_test");

    // ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
    if (!pcd_filename_indices.empty ())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile (filename, *point_cloud) == -1)
        {
            std::cout << "Was not able to open file \""<<filename<<"\".\n";
            printUsage (argv[0]);
        return 0;
        }
    }
    // pcl::io::loadPCDFile("test-mls.pcd",*cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(point_cloud);
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    viewer.runOnVisualizationThread(viewerPsycho);
    while(!viewer.wasStopped())
    {
        user_data++;
    }
    // ros::spin();

    return 0;
}

