#pragma once
#include "FeatureCloud.h"
#include "../MyPCLLib/GeneralTools.h"
#include <pcl/registration/lum.h>
#include <Eigen/StdVector>
#include <pcl/registration/gicp6d.h>
#include <pcl/visualization/pcl_visualizer.h>

//for Debug
#define DEBUG_OUT_DIR "C:/Code/PointCLoud/Data/filtered/filtered/outputsAll/debug/"
#define DEBUG_OUT_DIR2 "C:/Code/PointCLoud/Data/filtered/filtered/outputsAll/debug/oneToOne/"
#define DEBUG_LUM_VERTICES_DIR "C:/Code/PointCLoud/Data/AccurateRabbit/outputs/Debug/LUMVertices/"
#define DEBUG_ACCURATE_ALIGNMENT_DIR "C:/Code/PointCLoud/Data/AccurateRabbit/outputs/Debug/AccurateAlignment/"
#define VERY_BIG_FLOAT 1.0e20;
//#define DRAW_CORR true
//#define SAVE_LUM_VERTICES 1
//#define SAVE_ACCURATE_ALIGNMENT 1
 
class CloudAligner {
public:
	typedef pcl::PointXYZRGBNormal PType;
	typedef pcl::PointCloud<PType> PCType;
	typedef pcl::PointCloud<pcl::PointXYZRGB> OutPCType;

	typedef pcl::PointCloud<pcl::PointXYZ> BasicPCType;

	typedef pcl::PointCloud<pcl::Normal> NormalPCType;

public:
	CloudAligner() {};

	void setInputClouds(std::vector<PCType::Ptr> inputClouds, bool normalizeClouds = false);
	OutPCType::Ptr getAlignedCloud(int i) {
		Eigen::Affine3f Trans = lum.getTransformation(i);
		OutPCType::Ptr outCloud(new OutPCType);
		pcl::copyPointCloud(*lum.getPointCloud(i), *outCloud);
		pcl::transformPointCloud(*outCloud, *outCloud, Trans);
		return outCloud;
	}
	OutPCType::Ptr getConcatenatedCloud(){
		return lum.getConcatenatedCloud();
	}
	void makeAlignment();
	void makeFeatureClouds();

	bool useSimplifiedCloud = true;
	float simplificationVoxelSize = 0.1;
	bool esitmateNormals = false;
	bool normalEstimationRadius= 0.3;
	float fpfhRadius = 0.5;
	float corrsEstMaxDistance = 0.008;
	bool useSAC_IA = true;

	//sac_ia parameters
	float sac_ia_distance = 0.1;
	float sac_ia_max_correspondence_distance = 0.5;
	int sac_ia_max_iteration = 4000;

	//gicp6d parameters
	float gicp6d_maxCorrDistance = 0.5;
	int gicp6d_maxIteration = 5000;
	float gicp6d_L_weight = 0.01;
	//lum parameters
	int lum_maxIteration = 500;
	float lum_convergeThreshold = 0.01;

private:
	std::vector<PCType::Ptr> clouds;
	std::vector<FeatureCloud::Ptr> featureClouds;

	typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> TransVector;
	typedef std::vector<pcl::CorrespondencesPtr> CorrsVector;
	void makeRoughTrans(FeatureCloud::Ptr pTCloud, FeatureCloud::Ptr pSCloud, PCType::Ptr pOutCloud, Eigen::Affine3f &trans);
	void makeAccurateTrans(PCType::Ptr pTCloud, PCType::Ptr pSCloud, PCType::Ptr pOutCloud, Eigen::Affine3f &trans);
	void align2Clouds(FeatureCloud::Ptr pTCloud, FeatureCloud::Ptr pSCloud, Eigen::Affine3f &trans, pcl::CorrespondencesPtr pCorr);
	TransVector transformations;
	CorrsVector corrs;
	pcl::registration::LUM<pcl::PointXYZRGB> lum;
};

void CloudAligner::setInputClouds(std::vector<PCType::Ptr> inputClouds, bool normalizeClouds)
{
	clouds = inputClouds;
	if (normalizeClouds) {
		auto pFirstCloud = clouds.front();
		float scale = getAverageL2Norm(*pFirstCloud);
		for (auto pCloud : clouds) {
			normalizePointCloudUsingGivenScale(*pCloud, scale);
		}
	}
}

void CloudAligner::makeAlignment()
{
	makeFeatureClouds();
	int targetID, sourceID;

	for (int i = 0; i < clouds.size(); ++i) {
		targetID = i;
		if (i < clouds.size() - 1) {
			sourceID = i + 1;
		}
		else {
			sourceID = 0;
		}
		FeatureCloud::Ptr pTCloud = featureClouds[targetID], pSCloud = featureClouds[sourceID];

		Eigen::Affine3f trans;
		pcl::CorrespondencesPtr pCorr(new pcl::Correspondences);
		align2Clouds(pTCloud, pSCloud, trans, pCorr);
		transformations.push_back(trans);
		corrs.push_back(pCorr);
	}

	lum.setMaxIterations(lum_maxIteration);
	lum.setConvergenceThreshold(lum_convergeThreshold);

	std::cout << "Applying LUM." << std::endl;
	std::cout << "There is: " << corrs.size() << " correspondence." << std::endl;
	for (int i = 0; i < clouds.size(); ++i) {
		Eigen::Matrix4f combinedTrans;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::copyPointCloud(*clouds[i], *pTempCloud);
		pcl::copyPointCloud(*clouds[i], *pTempCloud);
		for (int j = i - 1; j >= 0; --j) {
			//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloudTemp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			combinedTrans =  transformations[j].matrix();
			pcl::transformPointCloud(*pTempCloud, *pTempCloud, combinedTrans, true);
		}

		cout << "Add to LUM pointCloud: " << i << endl;
#ifdef SAVE_LUM_VERTICES
		createDir(DEBUG_LUM_VERTICES_DIR);
		std::string outPath(DEBUG_LUM_VERTICES_DIR);
		std::cout << "Saving aligned file to: " << outPath << std::endl;
		std::stringstream ss;
		ss << outPath << i << ".pcd";
		pcl::io::savePCDFileBinary(ss.str(), *pTempCloud);
#endif
		lum.addPointCloud(pTempCloud);
		//getchar();
	}
	std::cout << "Start LUM computing." << std::endl;
	for (int i = 0; i < corrs.size(); ++i) {
		if (i < corrs.size() - 1) {
			lum.setCorrespondences(i, i+1, corrs[i]);
		}
		else {
			lum.setCorrespondences(corrs.size() - 1, 0, corrs.back());
#ifdef DRAW_CORR
			boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test")); //创建可视化窗口，名字叫做`test`
			view->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
			view->addCoordinateSystem(1.0); //建立空间直角坐标系
											//  viewer->setCameraPosition(0,0,200); //设置坐标原点
			view->initCameraParameters();   //初始化相机参数

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);

			pcl::copyPointCloud(*lum.getPointCloud(0), *pCloud0);
			pcl::copyPointCloud(*lum.getPointCloud(corrs.size() - 1), *pCloud1);

			Eigen::Affine3f trans = Eigen::Affine3f::Identity();
			trans.translation() << 0, 0, 1;
			pcl::transformPointCloud(*pCloud0, *pCloud0, trans);

			view->addPointCloud(pCloud0, "cloud0");
			view->addPointCloud(pCloud1, "cloud1");
			view->addCorrespondences<pcl::PointXYZRGB>(pCloud0, pCloud1, *corrs[i]);
			while (!view->wasStopped())
			{
				view->spinOnce(100);  //显示
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));   //随时间
			}

#endif
		}
	}

	lum.compute();
	std::cout << "Lum complete." << std::endl;

}

void CloudAligner::align2Clouds(FeatureCloud::Ptr pTCloud, FeatureCloud::Ptr pSCloud, Eigen::Affine3f &trans, pcl::CorrespondencesPtr pCorr) {
	Eigen::Affine3f roughTrans, accurateTrans, finalTrans;
	PCType::Ptr pTempCloud1(new PCType);
	PCType::Ptr pTempCloud2(new PCType);

	if (useSAC_IA){
		makeRoughTrans(pTCloud, pSCloud, pTempCloud1, roughTrans);
		makeAccurateTrans(pTCloud->getPCloud(), pTempCloud1, pTempCloud2, accurateTrans);
		trans = accurateTrans * roughTrans;
	}
	else {
		makeAccurateTrans(pTCloud->getPCloud(), pSCloud->getPCloud(), pTempCloud2, accurateTrans);
		trans = accurateTrans;
	}
	//estimate correspondence between target cloud and transformed source cloud,
	//corrs will be used by LUM algorithm
	std::cout << "Estimating correspondences." << std::endl;
	pcl::registration::CorrespondenceEstimation<PType, PType> ce;
	pcl::CorrespondencesPtr corr(new pcl::Correspondences);
	ce.setInputTarget(pTempCloud2);
	ce.setInputSource(pTCloud->getPCloud());
	ce.determineCorrespondences(*pCorr, corrsEstMaxDistance);
	std::cout << pCorr->size() << " correspondences find." << std::endl;

}

void CloudAligner::makeRoughTrans(FeatureCloud::Ptr pTCloud, FeatureCloud::Ptr pSCloud, PCType::Ptr pOutCloud, Eigen::Affine3f &trans){
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;

	int nr_iterations_(sac_ia_max_iteration);
	// Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance(sac_ia_distance);
	sac_ia_.setMaxCorrespondenceDistance(sac_ia_max_correspondence_distance);
	sac_ia_.setMaximumIterations(nr_iterations_);
	//sac_ia_.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//sac_ia_.setCorrespondenceRandomness(6);
	sac_ia_.setInputTarget(pTCloud->getPSimplifiedCloud());
	sac_ia_.setTargetFeatures(pTCloud->getPFpfhs());

	sac_ia_.setInputCloud(pSCloud->getPSimplifiedCloud());
	sac_ia_.setSourceFeatures(pSCloud->getPFpfhs());


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdToVis(new pcl::PointCloud<pcl::PointXYZRGB>);

	PCType::Ptr transformed_output(new PCType);
	BasicPCType registration_output;
	float best_score = VERY_BIG_FLOAT;

	std::cout << "Esimating transformation." << std::endl;
	sac_ia_.align(registration_output);
	best_score = sac_ia_.getFitnessScore();
	trans = sac_ia_.getFinalTransformation();

	printf("Fitness score: %f\n", sac_ia_.getFitnessScore());

	pcl::transformPointCloud(*pSCloud->getPCloud(), *pOutCloud, trans);

	////for Debug
	//std::string outPath(DEBUG_OUT_DIR);
	//std::cout << "Saving aligned file to: " << outPath << std::endl;
	//pcl::io::savePCDFileBinary(outPath + "aligned_rough.pcd", registration_output);
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pTempCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::copyPointCloud(*pOutCloud, *pTempCloud) ;
	//pcl::io::savePCDFileBinary(outPath + "aligned_rough_transformed.pcd", *pTempCloud);

	//pcl::io::savePCDFileBinary(outPath + "target_rough.pcd", *pTCloud->getPSimplifiedCloud());
	//getchar();

}

void CloudAligner::makeAccurateTrans(PCType::Ptr pTCloud, PCType::Ptr pSCloud, PCType::Ptr pOutCloud, Eigen::Affine3f &trans) {
	pcl::GeneralizedIterativeClosestPoint6D gicp6d(gicp6d_L_weight);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pSCloudRGB(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pTCloudRGB(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pFinalCloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

	pcl::copyPointCloud(*pSCloud, *pSCloudRGB);
	pcl::copyPointCloud(*pTCloud, *pTCloudRGB);

	gicp6d.setInputSource(pSCloudRGB);
	gicp6d.setInputTarget(pTCloudRGB);
	gicp6d.setMaxCorrespondenceDistance(gicp6d_maxCorrDistance);
	gicp6d.setMaximumIterations(gicp6d_maxIteration);
	gicp6d.align(*pFinalCloud);

	trans = gicp6d.getFinalTransformation();
	pcl::transformPointCloud(*pSCloud, *pOutCloud, trans);

	////for Debug
	static int i = 0;

#ifdef SAVE_ACCURATE_ALIGNMENT
	createDir(DEBUG_ACCURATE_ALIGNMENT_DIR);
	std::string outPath(DEBUG_ACCURATE_ALIGNMENT_DIR);
	std::cout << "Saving aligned file to: " << outPath << std::endl;
	std::stringstream ss;
	ss << outPath << i << "aligned.pcd";
	pcl::io::savePCDFileBinary(ss.str(), *pFinalCloud);

	ss.str("");
	ss << outPath << i << "aligned_transformed.pcd";
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pTempCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::transformPointCloud(*pSCloudRGB, *pTempCloud, trans);
	pcl::io::savePCDFileBinary(ss.str(), *pTempCloud);
	ss.str("");
	ss << outPath << i << "target.pcd";
	pcl::io::savePCDFileBinary(ss.str(), *pTCloudRGB);
	++i;
	//getchar();
#endif
}

inline void CloudAligner::makeFeatureClouds()
{
	featureClouds.clear();

	//make 
	for (auto pCloud : clouds) {
		FeatureCloud::Ptr pfc(new FeatureCloud);
		featureClouds.push_back(pfc);
		pfc->setInputClouds(pCloud);
		pfc->useSimplifiedCloud = useSimplifiedCloud;
		pfc->simplificationVoxelSize = simplificationVoxelSize;
		pfc->estimateNormals = esitmateNormals;
		pfc->normalEstimationRadius = normalEstimationRadius;
		pfc->fpfhRadius = fpfhRadius;

		pfc->calculateFeature();
	}
}
