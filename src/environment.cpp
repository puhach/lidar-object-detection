#include "sensors/lidar.h"
#include "render/render.h"
#include "point_cloud_processor.h"

#include <array>


void detectObstacles(pcl::visualization::PCLVisualizer::Ptr& viewer, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, PointCloudProcessor<pcl::PointXYZI> &pointProcessor)
{
        
    /// Measure obstacle detection process time
    //auto startTime = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.filterCloud(cloud, 0.2,
                                                                                    Eigen::Vector4f{ -5,-6,-3, 1 }, 
                                                                                    Eigen::Vector4f{+28,+7,+1, 1});
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>), roadCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::tie(obstacleCloud, roadCloud) = pointProcessor.segmentPlane(filteredCloud, 30, 0.3);

    renderPointCloud(viewer, roadCloud, "road", Color(0,1,0));
    //renderPointCloud(viewer, obstacleCloud, "obst", Color(1, 0, 0));

    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clusters = pointProcessor.clusterize(obstacleCloud, 0.6, 13, 3000);

    std::array<Color, 5> colors { Color(1,0,0), Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(0,0,1) };
    for (std::size_t i = 0; i < clusters.size(); ++i)
    {
        renderPointCloud(viewer, clusters[i], std::to_string(i), colors[i % colors.size()]);
        
        Box boundingBox = pointProcessor.boundingBox(clusters[i]);
        renderBox(viewer, boundingBox, i, Color(1,0,0), 0.5f);
    }

    //auto endTime = std::chrono::steady_clock::now();
    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "Obstacle detection took " << elapsedTime.count() << " milliseconds" << std::endl;

}   // detectObstacles

void initCamera(CameraAngle cameraAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    
    const int distance = 16; // distance away in meters
    
    switch(cameraAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(cameraAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}   // initCamera


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS; //Side; //XY;
    initCamera(setAngle, viewer);

    PointCloudProcessor<pcl::PointXYZI> pointProcessor;
    //std::vector<boost::filesystem::path> paths = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path> paths = pointProcessor.streamPcd("../data/pcd/data_1");
    std::vector<boost::filesystem::path>::const_iterator it = paths.cbegin();

    while (!viewer->wasStopped ())
    {
        
        if (it == paths.cend())
        {
            it = paths.cbegin();
            continue;
        }
        else
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pointProcessor.loadPcd(it->string());
            detectObstacles(viewer, cloud, pointProcessor);

            ++it;
        }   // it != paths.cend()

        //viewer->spinOnce();
        viewer->spinOnce(30);
    } // !viewer->wasStopped ()
}   // main