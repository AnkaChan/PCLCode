#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "../MyPCLLib/GeneralTools.h"
#include "../MyPCLLib/ioTools.h"
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <omp.h>
using std::cout;
using std::endl;
using std::string;
using std::vector;

#define APPROX_ZERO 10e-10

int
main(int argc, char** argv)
{
	if (argc < 3) {
		cout << "Please give a input dir and a output dir." << endl;
		return 0;
	}

	cout << "Input path: " << argv[1] << "\n" << "Output path: " << argv[2] << endl;

	boost::filesystem::path dir(argv[2]);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exists: " << argv[2] << std::endl;

		if (boost::filesystem::create_directory(dir))
			std::cout << "Successfully Created: " << argv[2] << std::endl;
	}

	vector<string> files;
	getJustCurrentFile(argv[1], files);
	string outPath(argv[2]);

	auto readPLYWrite2PCD = [&outPath](string &filePath){
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
		FileParts fp = fileparts(filePath);

		if (fp.ext == std::string(".ply") || fp.ext == std::string(".PLY")) {
			pcl::PLYReader Reader;
			Reader.read(filePath, *pCloud);

		}
		else if (fp.ext == std::string(".pcd") || fp.ext == std::string(".PCD")) {
			pcl::PCDReader Reader;
			Reader.read(filePath, *pCloud);
		}
		else {
			std::cout << "Unkown file format:\n: " << fp.ext << std::endl;
			return 2;
		}

		pcl::PointIndices::Ptr pIndicesToRemove(new pcl::PointIndices);
		pIndicesToRemove->indices.reserve(pCloud->points.size());
		int zeroCount = 0;
		#pragma omp parallel for
		for (int i = 0; i < pCloud->points.size(); ++i) {
			pcl::PointXYZRGBA& p = pCloud->points[i];
			if (abs(p.x) < APPROX_ZERO && abs(p.y) < APPROX_ZERO &&abs(p.z) < APPROX_ZERO) {
				++zeroCount;
			}
			else {
				pIndicesToRemove->indices.push_back(i);
			}
		}
		std::cout << "Zero points number: " << zeroCount << "\n";
		pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter(true); // Initializing with true will allow us to extract the removed indices
		eifilter.setInputCloud(pCloud);
		eifilter.setIndices(pIndicesToRemove);
		eifilter.filter(*pCloud_filtered);

		//pcl::io::savePCDFileASCII(outputName, cloud);
		string outputName = outPath + "\\" + fp.name + ".pcd";
		pcl::io::savePCDFileBinary(outputName, *pCloud_filtered);
		std::cout << "Saved " << pCloud_filtered->points.size() << " data points to: " << outputName << std::endl;
	};

	for_each(files.begin(), files.end(), readPLYWrite2PCD);

	return (0);
}