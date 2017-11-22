#pragma once

#include <limits>
#include <fstream>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <omp.h>
#include "CloudSimplifier.h"
class FeatureCloud {
public:
	typedef pcl::PointXYZRGBNormal PType;
	typedef pcl::PointCloud<PType> PCType;
	typedef pcl::PointXYZ BasicPType;
	typedef pcl::PointCloud<pcl::PointXYZ> BasicPCType;
	typedef pcl::PointCloud<pcl::Normal> NormalPCType;
	typedef typename pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> FPFHEstimator;
public:
	typedef boost::shared_ptr<FeatureCloud> Ptr;

	FeatureCloud() :
		fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>)
	{};
	FeatureCloud(PCType::Ptr new_cloud) :
		fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>)
	{
		setInputClouds(new_cloud);
	};
	void setInputClouds(PCType::Ptr new_cloud);

	PCType::Ptr getPCloud() {
		return pointCloud;
	}

	BasicPCType::Ptr getPSimplifiedCloud() {
		return simplifiedPointCloud;
	}

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getPFpfhs() {
		return fpfhs;
	}

	//Parameters
	bool useSimplifiedCloud = true;
	bool estimateNormals = false;
	float simplificationVoxelSize = 0.04;
	bool normalEstimationRadius = 0.3;
	float fpfhRadius = 0.5;

	void calculateFeature() {
		FPFHEstimator fpfh_estimate;
		int max_threadNum = omp_get_max_threads();
		fpfh_estimate.setNumberOfThreads(max_threadNum);

		//make simplifed cloud
		simplifiedPointCloud = BasicPCType::Ptr(new BasicPCType);
		if (useSimplifiedCloud) {
			CloudSimplifier<PType, pcl::PointXYZ> simplifier(pointCloud);
			simplifier.setVoxelSize(simplificationVoxelSize);
			simplifier.simplify(*simplifiedPointCloud);
		}
		else {
			pcl::copyPointCloud(*pointCloud, *simplifiedPointCloud);
		}

		//calculate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<BasicPType>::Ptr tree(new pcl::search::KdTree<BasicPType>());

		if (estimateNormals) {
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud(simplifiedPointCloud);
			ne.setSearchMethod(tree);
			ne.setRadiusSearch(normalEstimationRadius);
			ne.compute(*normals);
		}
		else {
			pcl::copyPointCloud(*simplifiedPointCloud, *normals);

		}
		fpfh_estimate.setInputCloud(simplifiedPointCloud);
		fpfh_estimate.setInputNormals(normals);
		fpfh_estimate.setSearchMethod(tree);
		//fpfh_estimate.setKSearch(fpfhKnn);
		fpfh_estimate.setRadiusSearch(fpfhRadius);

		fpfh_estimate.compute(*fpfhs);
	}
private:
	PCType::Ptr pointCloud;
	BasicPCType::Ptr simplifiedPointCloud;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs;


};

void FeatureCloud::setInputClouds(PCType::Ptr new_cloud)
{
	pointCloud = new_cloud;

}