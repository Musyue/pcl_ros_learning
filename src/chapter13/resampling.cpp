#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
int
main (int argc, char** argv)
{// ��һ���ʵ����͵������ļ����ص�����PointCloud��
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // ����bun0.pcd�ļ������ص��ļ��� PCL�Ĳ����������Ǵ��ڵ� 
  pcl::io::loadPCDFile ("pig.pcd", *cloud);
  // ����һ��KD��
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // ����ļ�����PointNormal���ͣ������洢�ƶ���С���˷�����ķ���
  pcl::PointCloud<pcl::PointNormal> mls_points;
  // ������� (�ڶ��ֶ���������Ϊ�˴洢����, ��ʹ�ò���Ҳ��Ҫ�������)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  //���ò���
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  // �����ؽ�
  mls.process (mls_points);
  // ������
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
}	
