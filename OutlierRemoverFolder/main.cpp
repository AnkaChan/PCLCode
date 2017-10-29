#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "../MyPCLLib/ioTools.h"
#include "../MyPCLLib/GeneralTools.h"

int
main (int argc, char** argv)
{
	if (argc != 3) {
		std::cout << "Please give a input directory and a output directory." << std::endl;
	}

	std::string inPath(argv[1]);
	std::string OutPath(argv[2]);

	std::vector<std::string> files; 
	get_filenames(inPath, files);

	OutPath = make2StandardPath(OutPath);
	createDir(OutPath);

	for (auto file : files) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

		FileParts fp = fileparts(file);
		// Fill in the cloud data
		pcl::PCDReader reader;
		// Replace the path below with the path where you saved your file
		reader.read<pcl::PointXYZRGB>(file, *cloud);

		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *cloud << std::endl;

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*cloud_filtered);

		std::cerr << "Cloud after filtering: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		pcl::PCDWriter writer;
		std::string outFile = OutPath + fp.name + fp.ext;
		std::cout << "Save to: " << outFile << std::endl;
		writer.write<pcl::PointXYZRGB>(outFile, *cloud_filtered, false);
	}

  return (0);
}