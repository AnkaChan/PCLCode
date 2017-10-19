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
#include "../MyPCLLib/ioTools.h"
#include "../MyPCLLib/GeneralTools.h"
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pSCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pTCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pFinalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

	loadPointCloud(argv[1], *pSCloud);
	loadPointCloud(argv[2], *pTCloud);

	pcl::GeneralizedIterativeClosestPoint6D gicp6d(0.024);
	gicp6d.setInputSource(pSCloud);
	gicp6d.setInputTarget(pSCloud);
	gicp6d.setMaxCorrespondenceDistance(0.2);
	gicp6d.setMaximumIterations(50);
	gicp6d.align(*pFinalCloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test")); //创建可视化窗口，名字叫做`test`
	view->addPointCloud(pFinalCloud, "filtered_cloud");
	view->initCameraParameters();   //初始化相机参数
	view->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
	view->addCoordinateSystem(1.0); //建立空间直角坐标系
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered_cloud");


	while (!view->wasStopped())
	{
		view->spinOnce(100);  //显示
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //随时间
	}

}