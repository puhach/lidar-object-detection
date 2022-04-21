
template <typename PointT>
bool EuclideanClusterer<PointT>::expandCluster(std::size_t pointIndex)
{
    auto clusterIndex = this->clusterAffinity[pointIndex];
    if (clusterIndex >= 0)
        return false;     // already in a cluster

    this->clusterAffinity[pointIndex] = numClusters;

    const auto &neighborIndices = this->tree.search(this->cloud->at(pointIndex), this->distanceTolerance);
    
    for (auto neighborIndex : neighborIndices)
    {
        expandCluster(neighborIndex);
    }

    return !neighborIndices.empty();
}   // expandCluster

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClusterer<PointT>::clusterize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
//std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClusterer<PointT>::clusterize(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    this->numClusters = 0;
    this->cloud = std::move(cloud);
    //KdTree<PointT> kdTree;
    this->tree.clear();

    //for (const auto& p : *cloud)
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        this->tree.insert(i, cloud->at(i));
    }

    //std::size_t numClusters = 0;
    //std::vector<std::make_signed_t<std::size_t>> clusterAffinity(cloud->size(), -1);
    this->clusterAffinity.resize(cloud->size());
    std::fill(this->clusterAffinity.begin(), this->clusterAffinity.end(), -1);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        if (expandCluster(i))
            ++this->numClusters;

    }   // for i


    assert(clusterAffinity.size() == cloud->size());
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;    // TODO: use numClusters
    for (std::size_t i = 0; i < this->clusterAffinity.size(); ++i)
    {
        auto clusterIndex = this->clusterAffinity[i];
        assert(clusterIndex >= 0);
        //auto& cluster = clusterIndex < clusters.size() ? clusters[clusterIndex] : clusters.emplace_back(new pcl::PointCloud<PointT>);
        if (clusterIndex >= clusters.size())
            clusters.emplace_back(new pcl::PointCloud<PointT>);
        clusters.back()->push_back(cloud->at(i));
    }   // for i

    return clusters;
}   // clusterize