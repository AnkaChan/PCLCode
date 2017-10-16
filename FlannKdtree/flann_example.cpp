#include <flann/flann.hpp>
#include <iostream>
//#include <flann/io/hdf5.h>
#include <ctime>
#include <random>
#include <stdio.h>
std::default_random_engine generator(time(NULL));
std::uniform_real_distribution<double> randReal(0.0, 1.0);

using namespace flann;
const int n = 100;
const int nDim = 3;

int main(int argc, char** argv)
{
	int nn = 3;

    Matrix<double> dataset(new double[nDim * n], n, nDim
	);
	for (size_t i = 0; i < n; i++)
	{
		dataset[i][0] = randReal(generator);
		dataset[i][1] = randReal(generator);
		dataset[i][2] = randReal(generator);

	}
    Matrix<int> indices(new int[n*nn], n, nn);
    Matrix<double> dists(new double[dataset.rows*nn], dataset.rows, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    Index<L2<double> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();                                                                                               

    // do a knn search, using 128 checks
    index.knnSearch(dataset, indices, dists, nn, flann::SearchParams(128));

	for (size_t i = 0; i < n; i++)
	{
		int id0 = indices[i][0];
		int id1 = indices[i][1];
		int id2 = indices[i][2];
		std::cout << "Point position:" << dataset[0][i] << ", " << dataset[1][i] << ", " << dataset[2][i] << "\n";
		std::cout << "Id0: " << id0 << " Id1: " << id1 << " Id2: " << id2 << std::endl;
		std::cout << "1st nearest point position:" << dataset[0][id0] << ", " << dataset[1][id0] << ", " << dataset[2][id0] << "\n";
		std::cout << "2nd nearest point position:" << dataset[0][id1] << ", " << dataset[1][id1] << ", " << dataset[2][id1] << "\n";
		std::cout << "3rd nearest point position:" << dataset[0][id2] << ", " << dataset[1][id2] << ", " << dataset[2][id2] << std::endl;
		getchar();
	}

    delete[] dataset.ptr();
    delete[] indices.ptr();
    delete[] dists.ptr();
    
    return 0;
}
