#ifndef EUCLIDEAN_CLUSTERER_H
#define EUCLIDEAN_CLUSTERER_H

#include <pcl/common/common.h>

#include "kdtree.h"

// TODO: test this adapter
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
		case 0: return point.x;
		case 1: return point.y;
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
	//constexpr XYZAdapter(T&& data) : data(std::move(data)) {}

	static constexpr std::size_t size() noexcept { return 3; }

	constexpr decltype(auto) operator [](std::size_t index) const
	{
		static_assert(std::extent<decltype(T::data)>::value >= XYZAdapter<T>::size(), "The data array must contain X, Y, Z coordinates.");
		return point.get().data[index];		
	}	// operator []

	std::size_t index;
	std::reference_wrapper<const T> point;
};	// XYZAdapter




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

	/*
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
	*/

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