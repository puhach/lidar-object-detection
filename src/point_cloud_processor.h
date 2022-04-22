#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H


#include <pcl/common/distances.h>   // pcl::PointCloud

#include <boost/filesystem.hpp>


template<typename PointT>
class PointCloudProcessor 
{
public:

    typename pcl::PointCloud<PointT>::Ptr filterCloud(const typename pcl::PointCloud<PointT>::ConstPtr & cloud, 
        float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlane(
        const typename pcl::PointCloud<PointT>::Ptr & cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterize(const typename pcl::PointCloud<PointT>::Ptr & cloud,
        float distanceTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterizePCL(typename pcl::PointCloud<PointT>::Ptr cloud,
        float distanceTolerance, int minSize, int maxSize);

    Box boundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};  // PointCloudProcessor


#include "point_cloud_processor.hpp"

#endif // POINT_CLOUD_PROCESSOR_H