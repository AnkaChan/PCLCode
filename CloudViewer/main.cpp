#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include "../MyPCLLib/GeneralTools.h"
#include "../MyPCLLib/ioTools.h"

#define APPROX_ZERO 10e-10

int 
main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Please give a input dir.\n" << std::endl;
        return 1;
    }
    else {
        std::cout << "Read: " << argv[1] << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

    FileParts fp = fileparts(argv[1]);

    if (fp.ext == std::string(".ply") || fp.ext == std::string(".PLY")) {
        pcl::PLYReader Reader;
        Reader.read(argv[1], *pCloud);

    }
    else if (fp.ext == std::string(".pcd") || fp.ext == std::string(".PCD")) {
        pcl::PCDReader Reader;
        Reader.read(argv[1], *pCloud);
    }
    else {
        std::cout << "Unkown file format:\n: " << fp.ext << std::endl;
        return 2;
    }
    
    //std::vector<int> indicesVector;
    pcl::PointIndices::Ptr pIndicesToRemove(new pcl::PointIndices);
    pIndicesToRemove->indices.reserve(pCloud->points.size());
    int zeroCount = 0;
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

	for (pcl::PointXYZRGBA & p : pCloud_filtered->points) {
		p.a = 255;
	}

    /*Eigen::Vector4f centroid;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    pcl::compute3DCentroid(*pCloud, centroid);
    trans.translation() << -centroid[0] , -centroid[1] , -centroid[2];
    pcl::transformPointCloud(*pCloud, *pCloud, trans);*/
    normalizePointCloudUsingAverageL2Norm(*pCloud_filtered);
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //
    ////blocks until the pCloud is actually rendered
    //viewer.showCloud(pCloud_filtered);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    //viewer.runOnVisualizationThread (viewerPsycho);
    //while (!viewer.wasStopped ())
    //{
    ////you can also do cool processing here
    ////FIXME: Note that this is running in a separate thread from viewerPsycho
    ////and you should guard against race conditions yourself...
    //
    //}

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test")); //创建可视化窗口，名字叫做`test`
    view->addPointCloud(pCloud_filtered, "filtered_cloud");
    view->initCameraParameters();   //初始化相机参数
    view->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
    view->addCoordinateSystem(1.0); //建立空间直角坐标系
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered_cloud");


    while (!view->wasStopped())
    {
        view->spinOnce(100);  //显示
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //随时间
    }

    return 0;
}