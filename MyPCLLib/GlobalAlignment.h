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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/lum.h>
#include <Eigen/StdVector>

#include "D:/Code/PointCloud/MyPCLLib/TemplateAlignment.h"

#define SQUARE(value_) ((value_)*(value_))

#define SAC_IA_MAX_ITERATION 500
#define ICP_NL_MAX_ITERATION 5000

#define SAC_IA_MAX_CORRESPONDANCE_D ((0.2*target.getScale())*(0.2*target.getScale()))

#define ICP_NL_MAX_CORRESPONDANCE_D (8.0)
#define SAC_IA_MAX_RETRY 3


namespace pcl
{

	Eigen::Matrix4f get_rough_align(FeatureCloud &target, FeatureCloud &input, FeatureCloud &output){
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

		float min_sample_distance_(0.1*target.getScale());
		float max_correspondence_distance_(SAC_IA_MAX_CORRESPONDANCE_D);
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
		reg.setTransformationEpsilon(0.000001);
		// Set the maximum distance between two correspondences (src<->tgt) to 10cm
		// Note: adjust this based on the size of your datasets
		reg.setMaxCorrespondenceDistance(ICP_NL_MAX_CORRESPONDANCE_D);
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
	void alignPointCloud(FeatureCloud &target, FeatureCloud &input, FeatureCloud &output, Eigen::Matrix4f &transformations_rough, Eigen::Matrix4f &transformations_precise){
		FeatureCloud roughAligned;
		cout << "Making rough alignment." << endl;
		transformations_rough = get_rough_align(target, input, roughAligned);
		cout << "Making precise alignment." << endl;
		transformations_precise = get_precise_align(target, roughAligned, output);
	}
}