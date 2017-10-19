#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <string>

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
	for (auto p : cloud.points) {
		Eigen::Vector3f v(p.x, p.y, p.z);
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
	scalePointCloudUsingGivenScale(cloud, 1/scale);
	return scale;
}

template <typename PointT>
float normalizePointCloudUsingGivenScale(pcl::PointCloud<PointT> & cloud, float scale) {
	Eigen::Vector4f centroid = pcl::compute3DCentroid(cloud);
	moveOnVector(cloud, -centroid);
	scalePointCloudUsingGivenScale(cloud, scale);
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
