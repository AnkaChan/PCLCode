#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
using std::string;
using std::vector;
using std::cout;
using std::endl;

void getJustCurrentFile(string path, vector<string>& files) {
	//�ļ����
	long long hFile = 0;
	//�ļ���Ϣ
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
	if (argc < 4)
	{
		cout << "Please input two directory and a leaf size parameter." << endl;
		return -1;
	}
	float leafSize = atof(argv[3]);
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
		//vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
		vox_grid.filter(*tempCloud);
		FileParts fp = filepartsWin(filePath);
		std::cout << "Saving simplified file to: " << outPath + fp.name + fp.ext << std::endl;
		pcl::io::savePCDFileBinary(outPath + fp.name + fp.ext, *tempCloud);

	}

	return 0;
}