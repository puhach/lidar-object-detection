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

	struct PointTreeItem
	{
		// PCL points seem to satisfy this requirement
		static_assert(std::is_array<decltype(PointT::data)>::value, "The point type must contain the data array.");

		PointTreeItem(std::size_t index, const PointT& point) : index(index), point(std::cref(point)) {}

		//PointTreeItem(std::size_t index, const PointT& point) : index(index), point(point) {}
		//PointTreeItem(std::size_t index, PointT&& point) : index(index), point(std::move(point)) {}
		
		//static constexpr std::size_t dim() noexcept { return sizeof(PointT::data) > 0 ? sizeof(PointT::data) / sizeof(PointT::data[0]) : 0; }
		static constexpr std::size_t dim() noexcept { return std::extent<decltype(PointT::data)>::value; }

		decltype(auto) operator [] (const std::size_t index) const { return point.get().data[index]; }

		std::size_t index;
		//PointT point;
		std::reference_wrapper<const PointT> point;
	};	// PointTreeItem

	//bool expandCluster(std::size_t pointIndex);
	bool expandCluster(const PointTreeItem &item);

	float distanceTolerance = 0;
	std::size_t minClusterSize = 0;
	std::size_t maxClusterSize = 0;
	typename pcl::PointCloud<PointT>::ConstPtr cloud;
	//KdTree<PointT> tree;
	KdTree<PointTreeItem> tree;
	std::vector<ClusterIndex> clusterAffinity;
	std::size_t numClusters;
};	// EuclideanClusterer

#include "euclidean_clusterer.hpp"

#endif	// EUCLIDEAN_CLUSTERER_H