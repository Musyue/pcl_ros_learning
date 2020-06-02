#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}

int
main(int argc, char *argv[])
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_xyz);

  const float min_scale = stof(argv[2]);          
  const int n_octaves = stof(argv[3]);            
  const int n_scales_per_octave = stof(argv[4]);  
  const float min_contrast = stof(argv[5]);       
 
  pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;//����sift�ؼ��������
  pcl::PointCloud<pcl::PointWithScale> result;
  sift.setInputCloud(cloud_xyz);//�����������
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
  sift.setSearchMethod(tree);//����һ���յ�kd������tree�����������ݸ�sift������
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);//ָ�������ؼ���ĳ߶ȷ�Χ
  sift.setMinimumContrast(min_contrast);//�������ƹؼ��������ֵ
  sift.compute(result);//ִ��sift�ؼ����⣬��������result

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(result, *cloud_temp);//��������pcl::PointWithScale������ת��Ϊ������pcl::PointXYZ������
 
  //���ӻ�������ƺ͹ؼ���
  pcl::visualization::PCLVisualizer viewer("Sift keypoint");
  viewer.setBackgroundColor( 255, 255, 255 );
  viewer.addPointCloud(cloud_xyz, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,0,"cloud");
  viewer.addPointCloud(cloud_temp, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"keypoints");

  while(!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return 0;
  
}