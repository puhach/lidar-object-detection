// PCL lib Functions for processing point clouds 

#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/common/distances.h>   // pcl::PointCloud

//#include <pcl/io/pcd_io.h>
//#include <pcl/common/common.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/common/transforms.h>
//#include <iostream> 
//#include <string>  
//#include <vector>
//#include <ctime>
//#include <chrono>
//#include "render/box.h"

#include <boost/filesystem.hpp>

template<typename PointT>
class PointCloudProcessor {
public:

    //PointCloudProcessor();
    //~PointCloudProcessor();

    //void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlane(
        const typename pcl::PointCloud<PointT>::Ptr & cloud, int maxIterations, float distanceThreshold);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane_Custom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);


    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterize(const typename pcl::PointCloud<PointT>::Ptr & cloud,
 
        float clusterTolerance, int minSize, int maxSize);


    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterizePCL(typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance, int minSize, int maxSize);


    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};


#include "point_cloud_processor.hpp"

#endif // POINT_CLOUD_PROCESSOR_H