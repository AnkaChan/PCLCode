#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

template <typename PointT, typename PointTOut>
class CloudSimplifier {
public:
	CloudSimplifier() {};
	CloudSimplifier(typename pcl::PointCloud<PointT>::Ptr new_inCloud)
	{
			setInputCloud(new_inCloud);
	};

	void setInputCloud(typename pcl::PointCloud<PointT>::Ptr new_inCloud) {
		inCloud = new_inCloud;
	}

	void simplify(pcl::PointCloud<PointTOut> & outCloud) {
		vox_grid.setInputCloud(inCloud);
		vox_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
		pcl::PointCloud<PointT> tempCloud;
		vox_grid.filter(tempCloud);
		pcl::copyPointCloud(tempCloud, outCloud);
	}

	void setVoxelSize(float size) {
		voxel_size = size;
	}
private:
	pcl::VoxelGrid<PointT> vox_grid;
	typename pcl::PointCloud<PointT>::Ptr inCloud;

	float voxel_size = 0.02;
};