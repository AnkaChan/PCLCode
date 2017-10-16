#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <pcl/registration/icp.h>
#include <pcl/features/fpfh.h>
#include "D:\Code\PointCloud\MyPCLLib\TemplateAlignment.h"
#include <pcl/visualization/pcl_visualizer.h>
//#define INDIR "D:/Code/PointCloud/Demos/data/demo1.pcd"
//#define INDIR0 "D:/Code/PointCloud/Sculpture/pcd/01.pcd"
//#define INDIR1 "D:/Code/PointCloud/Sculpture/pcd/03.pcd"
#define INDIR0 "D:/Code/PointCloud/Sculpture/pcd/demo1/00face.pcd"
#define INDIR1 "D:/Code/PointCloud/Sculpture/pcd/demo1/01face.pcd"

void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}
void
moveCentriod2Origin(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud){
	pcl::PointXYZ centriod(0.0, 0.0, 0.0);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::VectorType &data = pCloud->points;

	auto calculateCentriod = [&centriod](pcl::PointXYZRGBNormal p) {
		centriod.x += p.x;
		centriod.y += p.y;
		centriod.z += p.z;
	};
	std::for_each(data.begin(), data.end(), calculateCentriod);
	centriod.x /= pCloud->size();
	centriod.y /= pCloud->size();
	centriod.z /= pCloud->size();
	std::cout << "The centriod is: " <<
		centriod.x << ", " <<
		centriod.y << ", " <<
		centriod.z << std::endl;
	auto move2Origin = [centriod](pcl::PointXYZRGBNormal &p) {
		p.x -= centriod.x;
		p.y -= centriod.y;
		p.z -= centriod.z;
	};
	std::for_each(data.begin(), data.end(), move2Origin);
	std::cout << "Cloud has been moved." << std::endl;
}
void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	//user_data++;
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

int
main(int argc, char** argv)
{
	/*pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);*/
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	FeatureCloud featureCloud0, featureCloud1;
	std::vector<FeatureCloud> object_templates;

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(INDIR0, *cloud0) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << INDIR0 << "\n"
		<< cloud0->width * cloud0->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	moveCentriod2Origin(cloud0);
	featureCloud0.setInputCloud(cloud0);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(INDIR1, *cloud1) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded " << INDIR1 << "\n"
		<< cloud1->width * cloud1->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
	moveCentriod2Origin(cloud1);
	featureCloud1.setInputCloud(cloud1);

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

	float min_sample_distance_(0.2f);
	float max_correspondence_distance_(0.0000001f*0.0000001f);
	int nr_iterations_(500);
	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance(min_sample_distance_);
	sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
	sac_ia_.setMaximumIterations(nr_iterations_);
	sac_ia_.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia_.setCorrespondenceRandomness(6);
	sac_ia_.setInputTarget(featureCloud0.getPointCloudXYZ());
	sac_ia_.setTargetFeatures(featureCloud0.getLocalFeatures());

	sac_ia_.setInputCloud(featureCloud1.getPointCloudXYZ());
	sac_ia_.setSourceFeatures(featureCloud1.getLocalFeatures());


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pBaseViewPCD(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> registration_output, transformed_output;
	sac_ia_.align(registration_output);
	Eigen::Matrix3f rotation = sac_ia_.getFinalTransformation().block<3, 3>(0, 0);
	Eigen::Vector3f translation = sac_ia_.getFinalTransformation().block<3, 1>(0, 3);
	printf("Best fitness score: %f\n", sac_ia_.getFitnessScore());
	printf("Rotation is:");
	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	//pcl::transformPointCloud(registration_output, transformed_output, sac_ia_.getFinalTransformation());
	pcl::io::savePCDFileBinary("output.pcd", registration_output);
	pcl::PointCloud<pcl::PointXYZ> &cloud0_out = *featureCloud1.getPointCloudXYZ();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr pAligned(&registration_output);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pInput(featureCloud1.getPointCloudXYZ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pTarget(featureCloud0.getPointCloudXYZ());
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(pAligned, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(pTarget, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_cloud_color(pInput, 0, 0, 255);
	view->addPointCloud(pAligned, aligend_cloud_color, "aligend_cloud_v2");
	view->addPointCloud(pTarget, target_cloud_color, "target_cloud_v2");
	view->addPointCloud(pInput, input_cloud_color, "input_cloud_v2");

	pcl::io::savePCDFileBinary("target.pcd", cloud0_out);
	cout << "Aligned pcl has been saved." << endl;
	*pBaseViewPCD = cloud0_out + registration_output;
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(pcdToVis);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//viewer = simpleVis(pBaseViewPCD);
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	//viewer.runOnVisualizationThread(viewerPsycho);
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	system("pause");
	return (0);
}