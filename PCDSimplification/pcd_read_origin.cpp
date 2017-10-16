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

	float min_sample_distance_(0.05f);
	float max_correspondence_distance_(0.01f*0.01f);
	int nr_iterations_(500);
	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance(min_sample_distance_);
	sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
	sac_ia_.setMaximumIterations(nr_iterations_);

	sac_ia_.setInputTarget(featureCloud0.getPointCloudXYZ());
	sac_ia_.setTargetFeatures(featureCloud0.getLocalFeatures());

	sac_ia_.setInputCloud(featureCloud1.getPointCloudXYZ());
	sac_ia_.setSourceFeatures(featureCloud1.getLocalFeatures());


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ> registration_output, transformed_output;
	sac_ia_.align(registration_output);
	pcl::transformPointCloud(registration_output, transformed_output, sac_ia_.getFinalTransformation());
	pcl::io::savePCDFileBinary("output.pcd", transformed_output);
	pcl::PointCloud<pcl::PointXYZ> cloud0_out;
	copyPointCloud(*cloud0, cloud0_out);
	pcl::io::savePCDFileBinary("target.pcd", cloud0_out);
	cout << "Aligned pcl has been saved." << endl;
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(pcdToVis);
	//This will only get called once
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	//viewer.runOnVisualizationThread(viewerPsycho);

	getchar();
	return (0);
}