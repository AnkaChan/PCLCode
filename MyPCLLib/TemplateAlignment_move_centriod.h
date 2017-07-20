#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <algorithm>

class FeatureCloud
{
public:
	// A bit of shorthand
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloudBasic;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	FeatureCloud()
	{}

	~FeatureCloud() {}

	// Process the given cloud
	void
		setInputCloud(PointCloud::Ptr xyz)
	{
		originalPCL = xyz;
		xyz_ = PointCloudBasic::Ptr(new PointCloudBasic);
		copyPointCloud(*originalPCL, *xyz_);

		//if (scale_ == 0.0){
			moveCentriod2Origin();
			caculateScale();
		//}
		feature_radius_ = 0.5 * scale_;
		processInput();
	}

	// Load and process the cloud in the given PCD file
	void
		loadInputCloud(const std::string &pcd_file)
	{
		originalPCL = PointCloud::Ptr(new PointCloud);
		pcl::io::loadPCDFile(pcd_file, *originalPCL);
		xyz_ = PointCloudBasic::Ptr(new PointCloudBasic);
		copyPointCloud(*originalPCL, *xyz_);
		processInput();
	}

	// Get a pointer to the cloud 3D points
	PointCloud::Ptr
		getPointCloud() const
	{
		return (originalPCL);
	}
	// Get a pointer to the cloud 3D points with point type XYZ
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		getPointCloudXYZ() const
	{
		return (xyz_);
	}
	// Get a pointer to the cloud of 3D surface normals
	SurfaceNormals::Ptr
		getSurfaceNormals() const
	{
		return (normals_);
	}

	// Get a pointer to the cloud of feature descriptors
	LocalFeatures::Ptr
		getLocalFeatures() const
	{
		return (features_);
	}
	void
		moveCentriod2Origin(){
		pcl::PointXYZ centriod(0.0, 0.0, 0.0);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::VectorType &data = originalPCL->points;

		auto calculateCentriod = [&centriod](pcl::PointXYZRGBNormal p) {
			centriod.x += p.x;
			centriod.y += p.y;
			centriod.z += p.z;
		};
		std::for_each(data.begin(), data.end(), calculateCentriod);
		centriod.x /= originalPCL->size();
		centriod.y /= originalPCL->size();
		centriod.z /= originalPCL->size();
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
	float getScale(){
		return scale_;
	}

protected:
	// Compute the surface normals and local features
	void
		processInput()
	{
		computeSurfaceNormals();
		computeLocalFeatures();
	}

	// Compute the surface normals
	void
		computeSurfaceNormals()
	{
		normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
		copyPointCloud(*originalPCL, *normals_);
	}

	// Compute the local feature descriptors
	void
		computeLocalFeatures()
	{
		features_ = LocalFeatures::Ptr(new LocalFeatures);

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setInputCloud(xyz_);
		fpfh_est.setInputNormals(normals_);
		fpfh_est.setSearchMethod(search_method_xyz_);
		fpfh_est.setRadiusSearch(feature_radius_);
		fpfh_est.compute(*features_);
	}
	void 
		caculateScale()
	{
		float xNegativeSum = 0, xPositiveSum = 0;
		float yNegativeSum = 0, yPositiveSum = 0;
		float zNegativeSum = 0, zPositiveSum = 0;
		int xNegativeNum = 0, xPositiveNum = 0;
		int yNegativeNum = 0, yPositiveNum = 0;
		int zNegativeNum = 0, zPositiveNum = 0;
		float scaleSum = 0;
		
		pcl::PointCloud<pcl::PointXYZRGBNormal>::VectorType &data = originalPCL->points;
		auto getSums = 
			[&xNegativeSum, &xPositiveSum, &yNegativeSum, &yPositiveSum, &zNegativeSum, &zPositiveSum,
			&xNegativeNum, &xPositiveNum, &yNegativeNum, &yPositiveNum, &zNegativeNum, &zPositiveNum, &scaleSum](pcl::PointXYZRGBNormal p)
		{
			if (p.x > 0){
				xPositiveSum += p.x;
				++xPositiveNum;
			}
			else{
				++xNegativeNum;
				xNegativeSum += p.x;
			}
			if (p.y > 0){
				yPositiveSum += p.y;
				++yPositiveNum;
			}
			else{
				++yNegativeNum;
				yNegativeSum += p.y;
			}
			if (p.z > 0){
				zPositiveSum += p.z;
				++zPositiveNum;
			}
			else{
				++zNegativeNum;
				zNegativeSum += p.z;
			}

			scaleSum += sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
		};
		std::for_each(data.begin(), data.end(), getSums);
		x_scale_ = xPositiveSum / xPositiveNum - xNegativeSum / xNegativeNum;
		y_scale_ = yPositiveSum / yPositiveNum - yNegativeSum / yNegativeNum;
		z_scale_ = zPositiveSum / zPositiveNum - zNegativeSum / zNegativeNum;

		scale_ = scaleSum / data.size();
	}

private:
	// Point cloud data
	PointCloud::Ptr originalPCL;
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

	// Parameters
	float normal_radius_;
	float feature_radius_;
	static float scale_;
	float x_scale_;
	float y_scale_;
	float z_scale_;
};
float FeatureCloud::scale_ = 0.0;

