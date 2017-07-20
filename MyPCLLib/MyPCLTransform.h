#include <iostream>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/StdVector>

namespace pcl
{
	template<typename PointT> 
	class CMyTransformation{
	public:
		static void moveCentriod2Origin(pcl::PointCloud<PointT>::Ptr pCloud, Eigen::Vector4d &centroid){
			
			pcl::compute3DCentroid(*pCloud, centroid)

			std::cout << "The centriod is: " <<
				centriod[0] << ", " <<
				centriod[1] << ", " <<
				centriod[2] << std::endl;
			Eigen::Affine3f Trans = Eigen::Affine3f::Identity();
			Trans.translation() = -centriod;
			pcl::transformPointCloud(*pCloud, *pCloud, Trans);
			std::cout << "Cloud has been moved." << std::endl;
		}

		static void moveCentriod2Origin(pcl::PointCloud<PointT> &cloud, Eigen::Vector4d &centroid){

			pcl::compute3DCentroid(cloud, centroid)

				std::cout << "The centriod is: " <<
				centriod[0] << ", " <<
				centriod[1] << ", " <<
				centriod[2] << std::endl;
			Eigen::Affine3f Trans = Eigen::Affine3f::Identity();
			Trans.translation() = -centriod;
			pcl::transformPointCloud(cloud, cloud, Trans);
			std::cout << "Cloud has been moved." << std::endl;
		}

		static void moveCentriod2Origin(pcl::PointCloud<PointT>::Ptr pCloud){
			Eigen::Vector4d centroid;
			pcl::compute3DCentroid(*pCloud, centroid)

				std::cout << "The centriod is: " <<
				centriod[0] << ", " <<
				centriod[1] << ", " <<
				centriod[2] << std::endl;
			Eigen::Affine3f Trans = Eigen::Affine3f::Identity();
			Trans.translation() = -centriod;
			pcl::transformPointCloud(*pCloud, *pCloud, Trans);
			std::cout << "Cloud has been moved." << std::endl;
		}

		static void moveCentriod2Origin(pcl::PointCloud<PointT> &cloud){
			Eigen::Vector4d centroid;
			pcl::compute3DCentroid(cloud, centroid)

				std::cout << "The centriod is: " <<
				centriod[0] << ", " <<
				centriod[1] << ", " <<
				centriod[2] << std::endl;
			Eigen::Affine3f Trans = Eigen::Affine3f::Identity();
			Trans.translation() = -centriod;
			pcl::transformPointCloud(cloud, cloud, Trans);
			std::cout << "Cloud has been moved." << std::endl;
		}

		static void moveCentriod2Point(pcl::PointCloud<PointT> cloud, pcl::PointXYZ point){
			moveCentriod2Origin(cloud)

			Eigen::Affine3f Trans = Eigen::Affine3f::Identity();
			Trans.translation() << -point.x, -point.y, -point.z;
			pcl::transformPointCloud(*pCloud, *pCloud, Trans);
		}

		template<typename PointT>
		void transformWithNormal(pcl::PointCloud<PointT> &inputPointCloud, pcl::PointCloud<PointT> &outputPointCloud, const Eigen::Affine3f trans, bool copyAllFeild = true){
			pcl::PointCloud< PointT > normalSide;
			pcl::copyPointCloud(inputPointCloud, normalSide);
			pcl::PointCloud<PointT>::VectorType &pointsNormalSide = normalSide.points;
			for (size_t i = 0; i < pointsNormalSide.size(); ++i){
				PointT &p = pointsNormalSide[i];
				p.x += p.normal_x;
				p.y += p.normal_y;
				p.z += p.normal_z;
			}
			pcl::transformPointCloud(inputPointCloud, outputPointCloud, trans, copyAllFeild);
			pcl::transformPointCloud(normalSide, normalSide, trans, copyAllFeild);

			pcl::PointCloud<PointT>::VectorType &outPoints = outputPointCloud.points;
			for (size_t i = 0; i < outPoints.size(); ++i){
				PointT &p = outPoints[i];
				p.normal_x = pointsNormalSide[i].x - p.x;
				p.normal_y = pointsNormalSide[i].y - p.y;
				p.normal_z = pointsNormalSide[i].z - p.z;
			}
		}


	}
}
