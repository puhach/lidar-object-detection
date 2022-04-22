#ifndef EUCLIDEAN_CLUSTERER_H
#define EUCLIDEAN_CLUSTERER_H

#include "kdtree.h"

#include <pcl/common/common.h>


// A data adapter for indexing 2D data structures with X, Y fields
template <typename T>
struct XYAdapter
{
	constexpr XYAdapter(std::size_t index, const T& point) : index(index), point(std::cref(point)) {}

	static constexpr std::size_t size() noexcept { return 2; }

	constexpr decltype(auto) operator [](std::size_t index) const
	{
		switch (index)
		{
		case 0: return point.get().x;
		case 1: return point.get().y;
		default: throw std::out_of_range("Invalid coordinate index.");
		}
	}	// operator []

	std::size_t index;
	std::reference_wrapper<const T> point;
};	// XYAdapter


// A data adapter for indexing 3D data structures with X, Y, Z fields
template <typename T>
struct XYZAdapter
{
	// All PCL XYZ points seem to satisfy this requirement
	static_assert(std::is_array<decltype(T::data)>::value, "The point type must contain the data array.");

	constexpr XYZAdapter(std::size_t index, const T& point) : index(index), point(std::cref(point)) {}

	static constexpr std::size_t size() noexcept { return 3; }

	constexpr decltype(auto) operator [](std::size_t index) const
	{
		static_assert(std::extent<decltype(T::data)>::value >= XYZAdapter<T>::size(), "The data array must contain X, Y, Z coordinates.");
		return point.get().data[index];		
	}	// operator []

	std::size_t index;
	std::reference_wrapper<const T> point;
};	// XYZAdapter




// A class which performs Euclidean clustering of points
template <typename PointT>
class EuclideanClusterer
{
public:
	
	typedef std::make_signed_t<std::size_t> ClusterIndex;

	float getDistanceTolerance() const noexcept { return distanceTolerance; }
	void setDistanceTolerance(float distanceTolerance) noexcept { this->distanceTolerance = distanceTolerance; }

	std::size_t getMinClusterSize() const noexcept { return minClusterSize; }
	void setMinClusterSize(std::size_t minClusterSize) noexcept { this->minClusterSize = minClusterSize; }

	std::size_t getMaxClusterSize() const noexcept { return maxClusterSize; }
	void setMaxClusterSize(std::size_t maxClusterSize) noexcept { this->maxClusterSize = maxClusterSize; }

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterize(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

private:

	// Depending on the point type, organize them in 3D, 2D or KD space 

	template <typename T, typename X = void, typename Y = void, typename Z = void>
	struct AdapterSelector
	{
		using Type = T;
	};

	template <typename T, typename Z>
	struct AdapterSelector<T,	std::enable_if_t<std::is_arithmetic<decltype(T::x)>::value>,
								std::enable_if_t<std::is_arithmetic<decltype(T::y)>::value>, Z>
	{
		using Type = XYAdapter<T>;
	};

	template <typename T>
	struct AdapterSelector<T,	std::enable_if_t<std::is_arithmetic<decltype(T::x)>::value>,
								std::enable_if_t<std::is_arithmetic<decltype(T::y)>::value>,
								std::enable_if_t<std::is_arithmetic<decltype(T::z)>::value>>
	{
		using Type = XYZAdapter<T>;
	};


	typedef typename AdapterSelector<PointT>::Type PointTreeItem;

	bool expandCluster(const PointTreeItem &item);

	float distanceTolerance = 0;
	std::size_t minClusterSize = 0;
	std::size_t maxClusterSize = 0;
	typename pcl::PointCloud<PointT>::ConstPtr cloud;
	KdTree<PointTreeItem> tree;
	std::vector<ClusterIndex> clusterAffinity;
	std::size_t numClusters;
};	// EuclideanClusterer

#include "euclidean_clusterer.hpp"

#endif	// EUCLIDEAN_CLUSTERER_H