#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "../MyPCLLib//ioTools.h"
#include "../MyPCLLib//GeneralTools.h"
#include "CloudAligner.h"

using std::cout;
using std::endl;

using std::string;
using std::vector;

typedef pcl::PointXYZRGBNormal PType;
typedef pcl::PointCloud<PType> PCType;
typedef pcl::PointCloud<pcl::PointXYZ> BasicPCType;

int main(int argc, char ** argv) {
	if (argc < 3) {
		cout << "Please give original data's path and output data's path." << endl;
		return 1;
	}

	string inPath = make2StandardPath(argv[1]);
	string outPath = make2StandardPath(argv[2]);
	createDir(outPath);
	
	//load point clouds
	cout << "Input path: " << inPath << "\n" << "Output path: " << outPath << endl;
	vector<string> files, originalFiles;
	get_filenames(inPath, files);
	vector<PCType::Ptr> inputClouds;
	loadPointClouds<PCType>(files, inputClouds);

	//make feature clouds, which is to be processed by ia-ransac
	CloudAligner aligner;
	aligner.setInputClouds(inputClouds, true);
	aligner.useSAC_IA = true;
	aligner.esitmateNormals = true;
	aligner.useSimplifiedCloud = true;
	aligner.makeAlignment();
	aligner.useLum = false;
	aligner.gicp6d_color_weight = 0.05;

	string outFile;
	for (int i = 0; i < files.size(); ++i) {
		string path = files[i];
		FileParts fp = fileparts(path);
		CloudAligner::OutPCType::Ptr poutCloud = aligner.getAlignedCloud(i);

		outFile = outPath + fp.name + ".pcd";
		std::cout << "Saving aligned file to: " << outFile << std::endl;
		pcl::io::savePCDFileBinary(outFile, *poutCloud);
	}

	CloudAligner::OutPCType::Ptr poutCloud = aligner.getConcatenatedCloud();
	outFile = outPath + "ConcatenatedCloud.pcd";
	std::cout << "Saving aligned file to: " << outFile << std::endl;
	pcl::io::savePCDFileBinary(outFile, *poutCloud);
}