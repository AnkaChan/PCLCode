#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "./MeshLibIO.h"
#include "MeshLib/core/Parser/parser.h"
#include "../MyPCLLib/GeneralTools.h"
#include <pcl/filters/statistical_outlier_removal.h>

#define INDIR "D:/Code/PointCloud/Sculpture"
#define OUTDIR "./data"
CToolBox toolBox;
int
main(int argc, char** argv)
{
	if (argc < 3) {
		cout << "Please give a input dir and a output dir, if you want to remove outlier points, add \"R\" in the end." << endl;
		return 0;
	}

	cout << "Input path: " << argv[1] << "\n" << "Output path: " << argv[2] << endl;

	bool remove_outlier = false;
	if (argc == 4) {
		std::string s(argv[3]);
		if (s == "R" || s == "r") {
			remove_outlier = true;
		}
	}

	boost::filesystem::path dir(argv[2]);
	if (!(boost::filesystem::exists(dir))) {
		std::cout << "Doesn't Exists: " << argv[2] << std::endl;

		if (boost::filesystem::create_directory(dir))
			std::cout << "Successfully Created: " << argv[2] << std::endl;
	}

	vector<string> files;
	CToolBox::getJustCurrentFile(argv[1], files);
	string outPath(argv[2]);

	auto readMWrite2PCD = [&](string &filePath){
		CMesh mesh;
		readMFile(&mesh, filePath);
		cout << "Loaded: " << filePath << endl;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);


		// Fill in the cloud data
		cloud->width = mesh.numVertices();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		
		int i = 0;
		auto writePointData = [&cloud, &i](CVertexColor *pV){
			cloud->points[i].x = pV->point()[0];
			cloud->points[i].y = pV->point()[1];
			cloud->points[i].z = pV->point()[2];

			cloud->points[i].r = 255*pV->colorsRGB[0];
			cloud->points[i].g = 255*pV->colorsRGB[1];
			cloud->points[i].b = 255*pV->colorsRGB[2];

			cloud->points[i].normal_x = pV->normal[0];
			cloud->points[i].normal_y = pV->normal[1];
			cloud->points[i].normal_z = pV->normal[2];
			++i;
		};
		toolBox.MeshVertexIterFunc(&mesh, writePointData);

		CToolBox::FileParts fp = CToolBox::filepartsWin(filePath);
		if (*(outPath.end() - 1) != '\\' && *(outPath.end() - 1) != '/')
			outPath += '/';
		string outputName = outPath + fp.name + ".pcd";

		if (remove_outlier) {
			std::cout << "Filtering points." << std::endl;
			// Create the filtering object
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
			sor.setInputCloud(cloud);
			sor.setMeanK(50);
			sor.setStddevMulThresh(1.0);
			sor.filter(*filtered_cloud);

			//pcl::io::savePCDFileASCII(outputName, cloud);
			pcl::io::savePCDFileBinary(outputName, *filtered_cloud);
			std::cout << cloud->points.size() - filtered_cloud->points.size() << " points been deleted.\n";
			std::cout << "Saved " << filtered_cloud->points.size() << " data points to: " << outputName << std::endl;
		}
		else {
			//pcl::io::savePCDFileASCII(outputName, cloud);
			pcl::io::savePCDFileBinary(outputName, *cloud);
			std::cout << "Saved " << cloud->points.size() << " data points to: " << outputName << std::endl;
		}
		
	};

	for_each(files.begin(), files.end(), readMWrite2PCD);

	return (0);
}