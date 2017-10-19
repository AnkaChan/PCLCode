#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/lum.h>
#include <Eigen/StdVector>
#include "D:/Code/PointCloud/MyPCLLib/ioTools.h"

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pSCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pTCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());


}