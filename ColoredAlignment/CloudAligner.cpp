#include "CloudAligner.h"
#include "../MyPCLLib/GeneralTools.h"

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
	featureClouds.clear();
	for (auto pCloud : clouds) {
		FeatureCloud::Ptr pfc(new FeatureCloud);
		featureClouds.push_back(pfc);
		pfc->setInputClouds(pCloud);
		pfc->useSimplifiedCloud = useSimplifiedCloud;
		pfc->simplificationVoxelSize = simplificationVoxelSize;
		pfc->estimateNormals = esitmateNormals;
		pfc->normalEstimationRadius = normalEstimationRadius;
		pfc->fpfhKnn = fpfhKnn;

		pfc->calculateFeature();
	}
}
