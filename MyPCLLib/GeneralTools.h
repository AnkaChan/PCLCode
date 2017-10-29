#pragma once

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

template <typename PointT> 
void moveOnVector3Vec(pcl::PointCloud<PointT> & cloud, Eigen::Vector3f vec) {
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << vec[0], vec[1], vec[2];
	pcl::transformPointCloud(cloud, cloud, trans);
}

template <typename PointT>
void moveOnVector(pcl::PointCloud<PointT> & cloud, Eigen::Vector4f vec) {
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << vec[0], vec[1], vec[2];
	pcl::transformPointCloud(cloud, cloud, trans);
}

template <typename PointT>
float getAverageL2Norm(pcl::PointCloud<PointT> & cloud) {
	double totalNorm2 = 0;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud, centroid);
	for (auto p : cloud.points) {
		Eigen::Vector3f v(p.x - centroid[0], p.y - centroid[1], p.z - centroid[2]);
		totalNorm2 += v.norm();
	}
	return totalNorm2 / cloud.points.size();
}

template <typename PointT>
void scalePointCloudUsingGivenScale(pcl::PointCloud<PointT> & cloud, float scale) {
	for (PointT& p : cloud.points) {
		p.x *= scale; 
		p.y *= scale; 
		p.z *= scale;
	}
}

template <typename PointT>
float normalizePointCloudUsingAverageL2Norm(pcl::PointCloud<PointT> & cloud){
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud, centroid);
	moveOnVector(cloud, -centroid);
	float scale = getAverageL2Norm(cloud);
	scalePointCloudUsingGivenScale(cloud, 1 / scale);
	return scale;
}

template <typename PointT>
void normalizePointCloudUsingGivenScale(pcl::PointCloud<PointT> & cloud, float scale) {
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud, centroid);
	moveOnVector(cloud, -centroid);
	scalePointCloudUsingGivenScale(cloud, 1 / scale);
}

template <typename PointT>
bool loadPointCloud(std::string path, pcl::PointCloud<PointT> & cloud) {
	FileParts fp = fileparts(path);

	if (fp.ext == std::string(".ply") || fp.ext == std::string(".PLY")) {
		pcl::PLYReader Reader;
		Reader.read(path, cloud);
		return true;
	}
	else if (fp.ext == std::string(".pcd") || fp.ext == std::string(".PCD")) {
		pcl::PCDReader Reader;
		Reader.read(path, cloud);
		return true;
	}
	else {
		std::cout << "Unkown file format:\n: " << fp.ext << std::endl;
		return false;
	}
}
template <typename PointCloudT>
bool loadPointClouds(std::vector<std::string> paths, std::vector<boost::shared_ptr<PointCloudT>> & clouds) {
	clouds.reserve(paths.size());
	for (std::string filePath : paths) {
		PointCloudT::Ptr pCloud(new PointCloudT);

		if (!loadPointCloud(filePath, *pCloud)) {
			std::cout << "Skip file: " << filePath << std::endl;
			continue;
		}
		clouds.push_back(pCloud);
	}
	if (clouds.empty()) {
		return false;
	}
	else
	{
		return true;
	}
}