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
#include "D:/Code/PointCloud/MyPCLLib/TemplateAlignment.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/lum.h>
#include <Eigen/StdVector>
#include "D:/Code/PointCloud/MyPCLLib/ioTools.h"
#define SQUARE(value_) ((value_)*(value_))

#define SAC_IA_MAX_ITERATION 500
#define ICP_NL_MAX_ITERATION 5000

#define SAC_IA_MAX_CORRESPONDANCE_D ((0.2*target.getScale())*(0.2*target.getScale()))

#define ICP_NL_MAX_CORRESPONDANCE_D (8.0)
#define SAC_IA_MAX_RETRY 3

//#define INDIR "D:/Code/PointCloud/Demos/data/demo1.pcd"
//#define INDIR0 "D:/Code/PointCloud/Sculpture/pcd/01.pcd"
//#define INDIR1 "D:/Code/PointCloud/Sculpture/pcd/03.pcd"
//#define INDIR0 "D:/Code/PointCloud/Sculpture/pcd/01.pcd"
//#define INDIR1 "D:/Code/PointCloud/Sculpture/pcd/simplified/7/01.pcd"
#define INDIR0 "D:/Code/PointCloud/Sculpture/pcd/simplified/7/03.pcd"
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
moveCentriod2Origin(pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud){
	pcl::PointXYZ centriod(0.0, 0.0, 0.0);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::VectorType &data = Cloud.points;

	auto calculateCentriod = [&centriod](pcl::PointXYZRGBNormal p) {
		centriod.x += p.x;
		centriod.y += p.y;
		centriod.z += p.z;
	};
	std::for_each(data.begin(), data.end(), calculateCentriod);
	centriod.x /= Cloud.size();
	centriod.y /= Cloud.size();
	centriod.z /= Cloud.size();
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
pcl::PointXYZ
getCentriodPoint(pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud){
	pcl::PointXYZ centriod(0.0, 0.0, 0.0);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::VectorType &data = Cloud.points;

	auto calculateCentriod = [&centriod](pcl::PointXYZRGBNormal p) {
		centriod.x += p.x;
		centriod.y += p.y;
		centriod.z += p.z;
	};
	std::for_each(data.begin(), data.end(), calculateCentriod);
	centriod.x /= Cloud.size();
	centriod.y /= Cloud.size();
	centriod.z /= Cloud.size();
	std::cout << "The centriod is: " <<
		centriod.x << ", " <<
		centriod.y << ", " <<
		centriod.z << std::endl;
	return centriod;
}
void moveCentriod2Point(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud, pcl::PointXYZ point){
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
Eigen::Matrix4f get_rough_align(FeatureCloud &target, FeatureCloud &input, FeatureCloud &output){
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

	float min_sample_distance_(0.1*target.getScale());
	float max_correspondence_distance_( SAC_IA_MAX_CORRESPONDANCE_D );
	int nr_iterations_(SAC_IA_MAX_ITERATION);
	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance(min_sample_distance_);
	sac_ia_.setMaximumIterations(nr_iterations_);
	//sac_ia_.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//sac_ia_.setCorrespondenceRandomness(6);
	sac_ia_.setInputTarget(target.getPointCloudXYZ());
	sac_ia_.setTargetFeatures(target.getLocalFeatures());

	sac_ia_.setInputCloud(input.getPointCloudXYZ());
	sac_ia_.setSourceFeatures(input.getLocalFeatures());


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZ> registration_output;
	int failureTime = 0;
	float best_score = 100000000000000.0;
	Eigen::Matrix4f best_tranformation;
	do{
		sac_ia_.setMaximumIterations((1 + failureTime) * SAC_IA_MAX_ITERATION);

		cout << "Esimating transformation." << endl;
		sac_ia_.align(registration_output);
		if (sac_ia_.getFitnessScore() < best_score){
			best_score = sac_ia_.getFitnessScore();
			best_tranformation = sac_ia_.getFinalTransformation();
		}
			
		printf("Fitness score: %f\n", sac_ia_.getFitnessScore());
		++failureTime;
	} while (sac_ia_.getFitnessScore() > 160 && failureTime < SAC_IA_MAX_RETRY);
	
	Eigen::Matrix3f rotation = best_tranformation.block<3, 3>(0, 0);
	Eigen::Vector3f translation = best_tranformation.block<3, 1>(0, 3);
	printf("Best fitness score: %f\n", best_score);
	printf("Rotation is:");
	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	pcl::transformPointCloud(*input.getPointCloud(), *transformed_output, best_tranformation);
	
	output.setInputCloud(transformed_output);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh resualt"));
	//pcl::PointCloud<pcl::PointXYZ>::Ptr &pAligned = output.getPointCloudXYZ();
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pInput(input.getPointCloudXYZ());
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pTarget(target.getPointCloudXYZ());
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(pAligned, 255, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(pTarget, 0, 255, 0);
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_cloud_color(pInput, 0, 0, 255);
	//view->addPointCloud(pAligned, aligend_cloud_color, "aligend_cloud_v2");
	//view->addPointCloud(pTarget, target_cloud_color, "target_cloud_v2");
	////view->addPointCloud(pInput, input_cloud_color, "input_cloud_v2");
	//while (!view->wasStopped())
	//{
	//	view->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	return best_tranformation;
}

Eigen::Matrix4f get_precise_align(FeatureCloud &target, FeatureCloud &input, FeatureCloud &output){
	// Align
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> reg;
	reg.setTransformationEpsilon(0.00001);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(ICP_NL_MAX_CORRESPONDANCE_D );
	// Set the point representation
	//reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputCloud(input.getPointCloud());
	reg.setInputTarget(target.getPointCloud());

	reg.setMaximumIterations(ICP_NL_MAX_ITERATION);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reg_result(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	reg.align(*reg_result);
	cout << "The error of alignment: " << reg.getFitnessScore() << endl;

	output.setInputCloud(reg_result);
	return reg.getFinalTransformation();
}
Eigen::Matrix4f get_precise_align(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &output){
	// Align
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> reg;
	reg.setTransformationEpsilon(0.000001);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(ICP_NL_MAX_CORRESPONDANCE_D);
	// Set the point representation
	//reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputCloud(input);
	reg.setInputTarget(target);

	reg.setMaximumIterations(ICP_NL_MAX_ITERATION);

	reg.align(*output);
	cout << "The error of alignment: " << reg.getFitnessScore() << endl;

	return reg.getFinalTransformation();
}
void alignPointCloud(FeatureCloud &target, FeatureCloud &input, FeatureCloud &output, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transformations_rough, 
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &transformations_precise){
	FeatureCloud roughAligned;
	cout << "Making rough alignment." << endl;
	transformations_rough.push_back(get_rough_align(target, input, roughAligned));
	cout << "Making precise alignment." << endl;	
	transformations_precise.push_back(get_precise_align(target, roughAligned, output));
}

int
main(int argc, char** argv)
{
	bool plotBeforeAlign = false;
	bool plotAfterAlign = false;
	if (argc < 4){
		cout << "Please give simplied input path, output path and original data's path." << endl;
		return 0;
	}

	cout << "Input path: " << argv[1] << "\n" << "Output path: " << argv[2] << endl;
	vector<string> files, originalFiles;
	getJustCurrentFile(argv[1], files);
	getJustCurrentFile(argv[3], originalFiles);
	string outPath(argv[2]);
	if (*(outPath.end() - 1) != '\\' &&  *(outPath.end() - 1) != '/')
		outPath += "\\";

	
	FeatureCloud featureCloud0, featureCloud1, featureCloudAligned;
	
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformations_rough;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformations_precise;
	std::vector<pcl::CorrespondencesPtr> corrs;
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ce;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> pClouds;

	for (int i = 0; i < files.size(); ++i){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud0(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud0_o(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud1_o(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(files[i], *pCloud0) == -1) //* load the file
		{
			cout << "Couldn't read file " << files[i] << endl;
			return (-1);
		}
		cout << "Read file: " << files[i] << endl;
		moveCentriod2Origin(pCloud0);
		featureCloud0.setInputCloud(pCloud0);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(originalFiles[i], *pCloud0_o) == -1) //* load the file
		{
			cout << "Couldn't read file " << originalFiles[i] << endl;
			return (-1);
		}
		cout << "Read file: " << originalFiles[i] << endl;
		moveCentriod2Origin(pCloud0_o);
		pClouds.push_back(*pCloud0_o);

		string filePath;
		string filePath_o;
		if (i < files.size() - 1){
			filePath = files[i + 1];
			filePath_o = originalFiles[i + 1];
		}
		else{
			filePath = files[0];
			filePath_o = originalFiles[0];
		}
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filePath, *pCloud1) == -1) //* load the file
		{
			cout << "Couldn't read file " << filePath << endl;
			return (-1);
		}
		cout << "Read file: " << filePath << endl;
		moveCentriod2Origin(pCloud1);
		featureCloud1.setInputCloud(pCloud1);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filePath_o, *pCloud1_o) == -1) //* load the file
		{
			cout << "Couldn't read file " << filePath << endl;
			return (-1);
		}
		cout << "Read file: " << filePath_o << endl;
		moveCentriod2Origin(pCloud1_o);

		//featureCloud1.moveCentriod2Point(featureCloud0.getCentriod());
		if (plotBeforeAlign)
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh resualt"));
			pcl::PointCloud<pcl::PointXYZ>::Ptr pInput(featureCloud1.getPointCloudXYZ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr pTarget(featureCloud0.getPointCloudXYZ());
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(pInput, 255, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(pTarget, 0, 255, 0);
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_cloud_color(pInput, 0, 0, 255);
			view->addPointCloud(pInput, aligend_cloud_color, "aligend_cloud_v2");
			view->addPointCloud(pTarget, target_cloud_color, "target_cloud_v2");
			//view->addPointCloud(pInput, input_cloud_color, "input_cloud_v2");
			while (!view->wasStopped())
			{
				view->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}

		alignPointCloud(featureCloud0, featureCloud1, featureCloudAligned, transformations_rough, transformations_precise);
		transformations_precise.pop_back();
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloudTemp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::transformPointCloud(*pCloud1_o, *pCloudTemp, transformations_rough.back());
		transformations_precise.push_back(get_precise_align(pCloud0_o, pCloudTemp, pCloud1_o));
		pcl::CorrespondencesPtr corr(new pcl::Correspondences);
		ce.setInputTarget(pCloud1_o);
		ce.setInputSource(pCloud0_o);
		ce.determineCorrespondences(*corr, 2.5);
		corrs.push_back(corr);

		//FileParts fp = filepartsWin(filePath);
		//std::cout << "Saving simplified file to: " << outPath + fp.name + fp.ext << std::endl;
		//pcl::io::savePCDFileBinary(outPath + fp.name + fp.ext, *featureCloudAligned.getPointCloud());
		if (plotAfterAlign)
		{
			boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh resualt"));
			pcl::PointCloud<pcl::PointXYZ>::Ptr &pAligned = featureCloudAligned.getPointCloudXYZ();
			pcl::PointCloud<pcl::PointXYZ>::Ptr pInput(featureCloud1.getPointCloudXYZ());
			pcl::PointCloud<pcl::PointXYZ>::Ptr pTarget(featureCloud0.getPointCloudXYZ());
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(pAligned, 255, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(pTarget, 0, 255, 0);
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_cloud_color(pInput, 0, 0, 255);
			view->addPointCloud(pAligned, aligend_cloud_color, "aligend_cloud_v2");
			view->addPointCloud(pTarget, target_cloud_color, "target_cloud_v2");
			//view->addPointCloud(pInput, input_cloud_color, "input_cloud_v2");
			while (!view->wasStopped())
			{
				view->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}
		featureCloud0.setInputCloud(featureCloud1.getPointCloud());
	}
	

	pcl::registration::LUM<pcl::PointXYZRGBNormal> lum;
	lum.setMaxIterations(100000);
	lum.setConvergenceThreshold(0.01f);
	
	cout << "There is: " << pClouds.size() << " data." << endl;
	cout << "Transforming clouds. Totally " << transformations_rough.size()
		<< " + " << transformations_precise.size() << " Transformation. " << endl;
	for (int i = 0; i <= pClouds.size() - 1; ++i){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pInCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		*pInCloud = pClouds[i];
		for (int j = i - 1; j >= 0; --j){
			//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloudTemp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			Eigen::Matrix4f combinedTrans = transformations_precise[j] * transformations_rough[j];
			pcl::transformPointCloud(*pInCloud, *pInCloud, combinedTrans);
		}
		cout << "Add to LUM pointCloud: " << i << endl;
		pClouds[i] = *pInCloud;
		lum.addPointCloud(pInCloud);
		//getchar();
	}
	cout << "Applying LUM." << endl;
	cout << "There is: " << corrs.size() << " correspondence." << endl;
	for (int i = 0; i < corrs.size(); ++i){
		
		if (i < corrs.size() - 1){
			lum.setCorrespondences(i, i+1, corrs[i]);
		}
		else{
			lum.setCorrespondences(corrs.size() - 1, 0, corrs[corrs.size() - 1]);
		}
		cout << "Correspondence: " << i << endl;
	}
	
	lum.compute();
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloudOut = lum.getConcatenatedCloud();
	cout << "Saving Concatenated file to: " << outPath << " output.pcd. " << endl;
	pcl::io::savePCDFileBinary(outPath + "output.pcd", *pCloudOut);
	for (size_t i = 0; i < lum.getNumVertices(); i++)
	{
		FileParts fp = filepartsWin(files[i]);
		
		std::string result_filename = outPath + fp.name + fp.ext ;
		std::cout << "Saving aligned file to: " << outPath + fp.name + fp.ext << std::endl;
		Eigen::Affine3f Trans = lum.getTransformation(i);
		pcl::PointCloud<pcl::PointXYZRGBNormal> tempCloud;
		pcl::transformPointCloud(pClouds[i], tempCloud, Trans);

		pcl::io::savePCDFileBinary(result_filename, pClouds[i]);
		//std::cout << "saving result to " << result_filename << std::endl;
	}
	
	getchar();
	return (0);
}