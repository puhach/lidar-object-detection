
template <typename PointT>
bool EuclideanClusterer<PointT>::expandCluster(const PointTreeItem &item)
{
    // Check whether the point already belongs to a cluster
    auto clusterIndex = this->clusterAffinity[item.index];
    if (clusterIndex >= 0)
        return false;     

    // Assign a cluster index to this point. The current cluster index is the same as the number of clusters we have completely expanded.
    this->clusterAffinity[item.index] = this->numClusters;

    // Find nearby points
    const auto& neighbors = this->tree.search(item, this->distanceTolerance);

    // Add neighboring points to the same cluster
    for (const auto &neighbor : neighbors)
    {
        expandCluster(neighbor);
    }

    return !neighbors.empty();
}   // expandCluster

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClusterer<PointT>::clusterize(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    this->numClusters = 0;
    this->cloud = cloud; 
    this->tree.clear();

    // Add all points to the tree
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        this->tree.insert(EuclideanClusterer<PointT>::PointTreeItem{ i, cloud->at(i) });
    }

    // Map each point to a cluster index
    this->clusterAffinity.resize(cloud->size());
    std::fill(this->clusterAffinity.begin(), this->clusterAffinity.end(), -1);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        EuclideanClusterer<PointT>::PointTreeItem item{ i, this->cloud->at(i) };
        if (expandCluster(item))
            ++this->numClusters;
    }   // for i

    // Group points by their cluster indices
    assert(clusterAffinity.size() == cloud->size());
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters(numClusters);
    std::generate(clusters.begin(), clusters.end(), []() { return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>); });
    for (std::size_t i = 0; i < this->clusterAffinity.size(); ++i)
    {
        auto clusterIndex = this->clusterAffinity[i];
        assert(clusterIndex >= 0);
        assert(clusterIndex < clusters.size());
        clusters[clusterIndex]->push_back(cloud->at(i));
    }   // for i

    // Remove clusters which are too small or too large
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
        [this](const auto& cluster)
        {
            return cluster->size() < this->minClusterSize || cluster->size() > this->maxClusterSize;
        }), clusters.end());

    return clusters;
}   // clusterize