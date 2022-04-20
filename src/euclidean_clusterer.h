#ifndef EUCLIDEAN_CLUSTERER_H
#define EUCLIDEAN_CLUSTERER_H

#include <pcl/common/common.h>

#include "kdtree.h"


template <typename PointT>
class EuclideanClusterer
{
public:
	
	typedef std::make_signed_t<std::size_t> ClusterIndex;

	float getDistanceTolerance() const { return distanceTolerance; }
	void setDistanceTolerance(float distanceTolerance) { this->distanceTolerance = distanceTolerance; }

	std::size_t getMinClusterSize() const { return minClusterSize; }
	void setMinClusterSize(std::size_t minClusterSize) { this->minClusterSize = minClusterSize; }

	std::size_t getMaxClusterSize() const { return maxClusterSize; }
	void setMaxClusterSize(std::size_t maxClusterSize) { this->maxClusterSize = maxClusterSize; }

	///std::vector<ClusterIndex> clusterize(const pcl::PointCloud<PointT>& cloud);
	//std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterize(typename pcl::PointCloud<PointT>::ConstPtr cloud);
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

private:

	bool expandCluster(std::size_t pointIndex);

	float distanceTolerance = 0;
	std::size_t minClusterSize = 0;
	std::size_t maxClusterSize = 0;
	typename pcl::PointCloud<PointT>::ConstPtr cloud;
	KdTree<PointT> points;
	std::vector<ClusterIndex> clusterAffinity;
	std::size_t numClusters;
};	// EuclideanClusterer

#include "euclidean_clusterer.hpp"

#endif	// EUCLIDEAN_CLUSTERER_H