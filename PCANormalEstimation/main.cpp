#include <iostream>
#include <pcl/common/common.h>

#include <pcl/common/centroid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "../MyPCLLib/GeneralTools.h"
#include "../MyPCLLib/ioTools.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBNormal PType;
typedef pcl::PointCloud<PType> PCloudType;
typedef pcl::Normal NType;
typedef pcl::PointCloud<NType> NCloudType;
typedef pcl::PointXYZ BasicPType;
typedef pcl::PointCloud<BasicPType> BasicPCloudType;


int main(int argc, char ** argv){

	if (argc < 2) {
		cout << "Please give a input path." << endl;
		return 1;
	}

	FileParts fp = fileparts(argv[1]);
	PCloudType::Ptr pCloud(new PCloudType);
	BasicPCloudType::Ptr pBCloud(new BasicPCloudType);
	NCloudType::Ptr pNCloud(new NCloudType);


	cout << "Load: " << argv[1] << endl;
	loadPointCloud(argv[1], *pCloud);
	//... read, pass in or create a point cloud ...

	normalizePointCloudUsingAverageL2Norm(*pCloud);
	pcl::copyPointCloud(*pCloud, *pBCloud);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<BasicPType, NType> ne;
	ne.setInputCloud(pBCloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<BasicPType>::Ptr tree(new pcl::search::KdTree<BasicPType>());
	ne.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.2);

	// Compute the features
	ne.compute(*pNCloud);
	pcl::copyPointCloud(*pNCloud, *pCloud);
	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	string outPathPCD = fp.path + fp.name + "Normal.pcd";
	string outPathPLY = fp.path + fp.name + "Normal.ply";

	pcl::io::savePCDFileBinary(outPathPCD, *pCloud);
	pcl::io::savePLYFileASCII(outPathPLY, *pCloud);

	cout << "Save cloud with normal to: " << outPathPCD << endl;
	cout << "Save cloud with normal to: " << outPathPLY << endl;

}