#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
using std::string;
using std::vector;
using std::cout;
using std::endl;

void getJustCurrentFile(string path, vector<string>& files) {
	//文件句柄
	long long hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				;
			}
			else {
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
struct FileParts
{
	std::string path;
	std::string name;
	std::string ext;
};

FileParts fileparts(std::string filename)
{
	int idx0 = filename.rfind("/");
	int idx1 = filename.rfind(".");

	FileParts fp;
	fp.path = filename.substr(0, idx0 + 1);
	fp.name = filename.substr(idx0 + 1, idx1 - idx0 - 1);
	fp.ext = filename.substr(idx1);

	return fp;
}
FileParts filepartsWin(std::string filename)
{
	int idx0 = filename.rfind("\\");
	int idx1 = filename.rfind(".");

	FileParts fp;
	fp.path = filename.substr(0, idx0 + 1);
	fp.name = filename.substr(idx0 + 1, idx1 - idx0 - 1);
	fp.ext = filename.substr(idx1);

	return fp;
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		cout << "Please give a input dir, a output dir and a leaf size parameter(default 5)." << endl;
		return -1;
	}

	boost::filesystem::path dir(argv[2]);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exists: " << argv[2] << std::endl;

		if (boost::filesystem::create_directory(dir))
			std::cout << "Successfully Created: " << argv[2] << std::endl;
	}
	float leafSize = 5;
	if (argc == 4) {
		leafSize = atof(argv[3]);
	}
	cout << "Leaf size is: " << leafSize << endl;
	cout << "Input path: " << argv[1] << "\n" << "Output path: " << argv[2] << endl;
	vector<string> files;
	getJustCurrentFile(argv[1], files);
	string outPath(argv[2]);
	outPath += "\\";

	pcl::VoxelGrid<pcl::PointXYZRGBNormal> vox_grid;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	for (int i = 0; i < files.size(); ++i){
		string filePath = files[i];
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(filePath, *pCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file this pcd file. \n");
			return (-1);
		}
		const float voxel_grid_size = leafSize;

		vox_grid.setInputCloud(pCloud);
		vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);

		vox_grid.filter(*tempCloud);
		FileParts fp = filepartsWin(filePath);
		std::cout << "Saving simplified file to: " << outPath + fp.name + fp.ext << std::endl;
		pcl::io::savePCDFileBinary(outPath + fp.name + fp.ext, *tempCloud);

	}

	return 0;
}