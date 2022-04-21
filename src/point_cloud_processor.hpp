// PCL lib Functions for processing point clouds 

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

//#include "point_cloud_processor.h"

#include <unordered_set>
//#include "point_cloud_processor.h"
#include "euclidean_clusterer.h"

//#include <pcl/filters/random_sample.h>

//#include <pcl/types.h>

//constructor:
//template<typename PointT>
//PointCloudProcessor<PointT>::PointCloudProcessor() {}
//
//
////de-constructor:
//template<typename PointT>
//PointCloudProcessor<PointT>::~PointCloudProcessor() {}


//template<typename PointT>
//void PointCloudProcessor<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
//{
//    std::cout << cloud->points.size() << std::endl;
//}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudProcessor<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
                                                                            float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*filteredCloud);

    pcl::CropBox<PointT> cropBox;
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*filteredCloud);

    //pcl::PointIndices::Ptr roofIndices(new pcl::PointIndices);
    pcl::IndicesPtr roofIndices(new pcl::Indices);
    pcl::CropBox<PointT> roofBox;
    roofBox.setMin(Eigen::Vector4f{ -2, -2, -1, 1 });
    roofBox.setMax(Eigen::Vector4f{ +3, +2, 1, 1 });
    roofBox.setInputCloud(filteredCloud);
    roofBox.filter(*roofIndices);

    pcl::ExtractIndices<PointT> extractor;
    extractor.setIndices(roofIndices);
    extractor.setNegative(true);
    extractor.setInputCloud(filteredCloud);
    extractor.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


//template<typename PointT>
//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
//{
//    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
//
//    pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>()), roadCloud(new pcl::PointCloud<PointT>());
//
//    pcl::ExtractIndices<PointT> extractor;
//    extractor.setInputCloud(cloud);
//    extractor.setNegative(false);
//    extractor.setIndices(inliers);
//    extractor.filter(*roadCloud);
//    
//    extractor.setNegative(true);
//    extractor.filter(*obstacleCloud);
//
//    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
//    return segResult;
//}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::segmentPlane(
    const typename pcl::PointCloud<PointT>::Ptr & cloud, int maxIterations, float distanceThreshold)
{
    
    // A helper function for sampling n random points from the cloud
    auto sample = [](const pcl::PointCloud<PointT>::Ptr& cloud, std::size_t n)
    {
        typename pcl::PointCloud<PointT>::Ptr sampledCloud(new pcl::PointCloud<PointT>());
        std::unordered_set<int> sampledIndices;

        if (cloud->size() < n)  // check whether it is possible to sample n different points from this cloud
            return sampledCloud;

        // Repeat until n different points are sampled (we may need to do this more than n times, because there is a chance
        // that the same point is sampled several times)
        while (sampledIndices.size() < n)   
        {
            int index = rand() % cloud->size();
            sampledIndices.insert(index);
        }

        // Populate the output cloud with the points at the indices that we have selected
        for (auto index : sampledIndices)   
            sampledCloud->points.push_back(cloud->at(index));

        return sampledCloud;
    };  // sample


    // Initialize random number generation once for each thread (srand is not guaranteed to be thread-safe)
    thread_local int dummy = (std::srand(std::time(0)), 0);  

    std::unordered_set<std::size_t> bestInliers;    // indices of the inliers for the best fitted plane

    // Try fitting a plane to different sets of points maxInterations times
    for (int i = 0; i < maxIterations; ++i)     
    {
        // Randomly sample subset and fit line
        typename pcl::PointCloud<PointT>::Ptr sampledCloud = sample(cloud, 3);
        assert(sampledCloud->size() == 3);

        // It works with PCL 1.12 and MSVC, but not with PCL 1.7 and GCC
        //pcl::PointCloud<PointT> sampledCloud;
        //pcl::RandomSample<PointT> sampler;
        //sampler.setInputCloud(cloud);
        //sampler.setSample(3);
        ////sampler.setSeed(123);
        //sampler.filter(sampledCloud);
        //assert(sampledCloud.size() == 3);

        // Fit a plane to the 3 points we have sampled
        auto x1 = sampledCloud->at(0).x, y1 = sampledCloud->at(0).y, z1 = sampledCloud->at(0).z;
        auto x2 = sampledCloud->at(1).x, y2 = sampledCloud->at(1).y, z2 = sampledCloud->at(1).z;
        auto x3 = sampledCloud->at(2).x, y3 = sampledCloud->at(2).y, z3 = sampledCloud->at(2).z;
        auto a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (z3 - y1);
        auto b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        auto c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        auto d = -(a * x1 + b * y1 + c * z1);

        // Compute the squared magnitude of the normal vector
        auto magN2 = a * a + b * b + c * c;  

        if (magN2 < std::numeric_limits<decltype(magN2)>::epsilon())  // prevent division by zero
            continue;

        // Measure the distance between every point and the fitted plane. If the distance is smaller than threshold, 
        // count it as an inlier.
        //inliers.clear();
        std::unordered_set<std::size_t> inliers;
        for (std::size_t j = 0; j < cloud->size(); ++j)
        {
            PointT& p = cloud->at(j);
            auto x0 = p.x, y0 = p.y, z0 = p.z;

            auto dot = a * x0 + b * y0 + c * z0 + d;
            auto dist2 = dot * dot / magN2;
            
            if (dist2 < distanceThreshold * distanceThreshold)
                inliers.insert(j);

            //double dist = std::abs(a * x0 + b * y0 + c * z0 + d) / std::sqrt(a * a + b * b + c * c);
            //if (dist < distanceThreshold)
            //    inliers.insert(j);
        }   // for j

        if (inliers.size() > bestInliers.size())
            inliers.swap(bestInliers);
    }   // for i

    // Create separate clouds for the plane (road) and other points (obstacles)
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>()), obstacleCloud(new pcl::PointCloud<PointT>());
    for (int i = 0; i < cloud->size(); ++i)
    {
        if (bestInliers.find(i) == bestInliers.cend())
            obstacleCloud->points.push_back(cloud->at(i));
        else
            roadCloud->points.push_back(cloud->at(i));
    }

    return std::make_pair(std::move(obstacleCloud), std::move(roadCloud));
}   // segmentPlane


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::clusterize(
    const typename pcl::PointCloud<PointT>::Ptr & cloud, float distanceTolerance, int minSize, int maxSize)
{
    // Measure clustering process time
    auto startTime = std::chrono::steady_clock::now();

    // Perform euclidean clustering to group detected obstacles

    EuclideanClusterer<PointT> clusterer; 
    clusterer.setDistanceTolerance(distanceTolerance);
    clusterer.setMinClusterSize(minSize);
    clusterer.setMaxClusterSize(maxSize);
    
    auto clusters = clusterer.clusterize(cloud);

    //auto pointClusterAffinity = clusterer.clusterize(*cloud);

    //assert(pointClusterAffinity.size() == cloud->size());
    //std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    //for (std::size_t i = 0; i < pointClusterAffinity.size(); ++i)
    //{
    //    auto clusterIndex = pointClusterAffinity[i];
    //    assert(clusterIndex >= 0);
    //    auto& cluster = clusterIndex < clusters.size() ? clusters[clusterIndex] : clusters.emplace_back(new pcl::PointCloud<T>);
    //    cluster->push_back(cloud->at(i));
    //}   // for i



    
        /*
    pcl::EuclideanClusterExtraction<PointT> euclideanClusterExtraction;
    euclideanClusterExtraction.setDistanceTolerance(distanceTolerance);
    euclideanClusterExtraction.setMinClusterSize(minSize);
    euclideanClusterExtraction.setMaxClusterSize(maxSize);
    euclideanClusterExtraction.setSearchMethod(kdTree);
    euclideanClusterExtraction.setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    euclideanClusterExtraction.extract(clusterIndices);
   

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters(clusterIndices.size());
    std::transform(clusterIndices.cbegin(), clusterIndices.cend(), clusters.begin(),
        [&cloud](const pcl::PointIndices& pointIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (pcl::index_t pointIndex : pointIndices.indices)
            {
                cluster->push_back(cloud->at(pointIndex));
            }

            return cluster;
        });
        */

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}   // clusterize

template<typename PointT>
inline std::vector<typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::clusterizePCL(
    typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, int minSize, int maxSize)
{
    // Measure clustering process time
    auto startTime = std::chrono::steady_clock::now();

    // Perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());

    pcl::EuclideanClusterExtraction<PointT> euclideanClusterExtraction;
    euclideanClusterExtraction.setDistanceTolerance(distanceTolerance);
    euclideanClusterExtraction.setMinClusterSize(minSize);
    euclideanClusterExtraction.setMaxClusterSize(maxSize);
    euclideanClusterExtraction.setSearchMethod(kdTree);
    euclideanClusterExtraction.setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    euclideanClusterExtraction.extract(clusterIndices);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters(clusterIndices.size());
    std::transform(clusterIndices.cbegin(), clusterIndices.cend(), clusters.begin(),
        [&cloud](const pcl::PointIndices& pointIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (pcl::index_t pointIndex : pointIndices.indices)
            {
                cluster->push_back(cloud->at(pointIndex));
            }

            return cluster;
        });

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}   // clusterizePCL


template<typename PointT>
Box PointCloudProcessor<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void PointCloudProcessor<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudProcessor<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> PointCloudProcessor<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}