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

	if (argc != 4) {
		std::cout << "Please give two input dirs and a output dir." << std::endl;
		return 1;
	}

	std::cout << "load: " << argv[1] << std::endl;
	loadPointCloud(argv[1], *pSCloud);
	std::cout << "load: " << argv[2] << std::endl;
	loadPointCloud(argv[2], *pTCloud);
	/*float scale = normalizePointCloudUsingAverageL2Norm(*pSCloud);
	normalizePointCloudUsingGivenScale(*pTCloud, scale);*/

	pcl::GeneralizedIterativeClosestPoint6D gicp6d(0.05);
	gicp6d.setInputSource(pSCloud);
	gicp6d.setInputTarget(pTCloud);
	gicp6d.setMaxCorrespondenceDistance(0.3);
	gicp6d.setMaximumIterations(50);
	gicp6d.align(*pFinalCloud);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test")); //创建可视化窗口，名字叫做`test`
	view->addPointCloud(pFinalCloud, "aligned_cloud");
	view->addPointCloud(pTCloud, "target_cloud");

	view->initCameraParameters();   //初始化相机参数
	view->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
	view->addCoordinateSystem(1.0); //建立空间直角坐标系
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "aligned_cloud");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "target_cloud");

	while (!view->wasStopped())
	{
		view->spinOnce(100);  //显示
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //随时间
	}

	FileParts fp1 = fileparts(argv[1]);
	FileParts fp2 = fileparts(argv[2]);

	std::string outputDir(argv[3]);
	outputDir = make2StandardPath(outputDir);
	createDir(outputDir);

	std::string outpath1 = outputDir + fp1.name + ".pcd";
	std::string outpath2 = outputDir + fp2.name + ".pcd";

	pcl::io::savePCDFileBinary(outpath1, *pFinalCloud);
	std::cout << "Saved data points to: " << outpath1 << std::endl;

	pcl::io::savePCDFileBinary(outpath2, *pTCloud);
	std::cout << "Saved data points to: " << outpath2 << std::endl;

}