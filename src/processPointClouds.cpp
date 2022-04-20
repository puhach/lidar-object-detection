// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>
#include <pcl/filters/random_sample.h>

//#include <pcl/types.h>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, 
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>()), roadCloud(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setNegative(false);
    extractor.setIndices(inliers);
    extractor.filter(*roadCloud);
    
    extractor.setNegative(true);
    extractor.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> segmenter;
    segmenter.setOptimizeCoefficients(true);    // optional
    segmenter.setModelType(pcl::SACMODEL_PLANE);
    segmenter.setMethodType(pcl::SAC_RANSAC);
    segmenter.setMaxIterations(maxIterations);
    segmenter.setDistanceThreshold(distanceThreshold);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients());
    segmenter.setInputCloud(cloud);
    segmenter.segment(*inliers, *coefs);

    /*
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients());

    auto ratio = 0.3;
    auto n = cloud->size();
    while (cloud->size() > ratio * n)
    {
        pcl::PointIndices::Ptr curInliers(new pcl::PointIndices());
        segmenter.setInputCloud(cloud);
        segmenter.segment(*curInliers, *coefs);

        if (curInliers->indices.empty())
        {
            break;
        }

        inliers->indices.insert(inliers->indices.end(), curInliers->indices.begin(), curInliers->indices.end());

        pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>());
        extractor.setInputCloud(cloud);
        extractor.setIndices(curInliers);
        extractor.setNegative(true);
        extractor.filter(*outliers);
        cloud.swap(outliers);
    }   // while
    */

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_Custom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    //auto sample = [](const auto& cloud, auto n)
    auto sample = [](const pcl::PointCloud<PointT>::Ptr &cloud, std::size_t n)
    {
        typename pcl::PointCloud<PointT>::Ptr sampledCloud(new pcl::PointCloud<PointT>());
        std::unordered_set<int> sampledIndices;

        if (cloud->size() < n)
            return sampledCloud;

        while (sampledIndices.size() < n)
        {
            int index = rand() % cloud->size();
            sampledIndices.insert(index);
        }

        for (auto index : sampledIndices)
            sampledCloud->points.push_back(cloud->at(index));

        return sampledCloud;
    };  // sample

    // For max iterations 
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit line
        typename pcl::PointCloud<PointT>::Ptr sampledCloud = sample(cloud, 3);
        assert(sampledCloud->size() == 3);
        
        //pcl::PointCloud<PointT> sampledCloud;
        //pcl::RandomSample<PointT> sampler;
        //sampler.setInputCloud(cloud);
        //sampler.setSample(3);
        ////sampler.setSeed(123);
        //sampler.filter(sampledCloud);
        //assert(sampledCloud.size() == 3);


        auto x1 = sampledCloud->at(0).x, y1 = sampledCloud->at(0).y, z1 = sampledCloud->at(0).z;
        auto x2 = sampledCloud->at(1).x, y2 = sampledCloud->at(1).y, z2 = sampledCloud->at(1).z;
        auto x3 = sampledCloud->at(2).x, y3 = sampledCloud->at(2).y, z3 = sampledCloud->at(2).z;
        auto a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (z3 - y1);
        auto b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        auto c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        auto d = -(a * x1 + b * y1 + c * z1);

        if (a * a + b * b + c * c < std::numeric_limits<float>::epsilon())  // prevent division by zero
            continue;

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        std::unordered_set<int> inliers;
        for (int j = 0; j < cloud->size(); ++j)
        {
            pcl::PointXYZ& p = cloud->at(j);
            auto x0 = p.x, y0 = p.y, z0 = p.z;

            double dist = std::abs(a * x0 + b * y0 + c * z0 + d) / std::sqrt(a * a + b * b + c * c);
            if (dist < distanceThreshold)
                inliers.insert(j);
        }   // for j

        if (inliers.size() > inliersResult.size())
            inliers.swap(inliersResult);
    }   // for i

    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>()), obstacleCloud(new pcl::PointCloud<PointT>());
    for (int i = 0; i < cloud->size(); ++i)
    {
        if (inliersResult.find(i) == inliersResult.cend())
            obstacleCloud->points.push_back(cloud->at(i));
        else
            roadCloud->points.push_back(cloud->at(i));
    }

    return std::make_pair(obstacleCloud, roadCloud);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    typename pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());

    pcl::EuclideanClusterExtraction<PointT> euclideanClusterExtraction;
    euclideanClusterExtraction.setClusterTolerance(clusterTolerance);
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
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}