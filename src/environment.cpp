/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "point_cloud_processor.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "processPointClouds.cpp"

#include <boost/format.hpp> // my

#include <array>


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    std::unique_ptr<Lidar> lidar(new Lidar(std::move(cars), 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
    //renderRays(viewer, lidar->position, pointCloud);
    //renderPointCloud(viewer, pointCloud, "some name", Color(0, 1.0, 0.5));

    // TODO: check whether the size is correctly obtained for XYZI and other point types
    //constexpr std::size_t sz = sizeof(pcl::PointXYZ::data) / sizeof(pcl::PointXYZ::data[0]);
    

    // Create point processor
    PointCloudProcessor<pcl::PointXYZ> pointProcessor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud, roadCloud;
    std::tie(obstacleCloud, roadCloud) = pointProcessor.segmentPlane(pointCloud, 100, 0.2);
    //std::tie(obstacleCloud, roadCloud) = pointProcessor.SegmentPlane_Custom(pointCloud, 50, 0.2);

    //renderPointCloud(viewer, obstacleCloud, "obstacleCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, roadCloud, "roadCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.clusterize(obstacleCloud, 1, 3, 30);
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,1,1) };
    for (std::size_t i = 0; i < clusters.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters[i];

        std::cout << "cluster size " << cluster->size() << std::endl; 
        //pointProcessor.numPoints(cluster);
        
        const Color& color = colors[i % colors.size()];
        renderPointCloud(viewer, cluster, boost::str(boost::format("cluster_%d") % i), color);

        Box boundingBox = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, boundingBox, i, Color(1,1,0));
    }
}   // simpleHighway

void detectObstacles(pcl::visualization::PCLVisualizer::Ptr& viewer, 
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, PointCloudProcessor<pcl::PointXYZI> &pointProcessor)
{
        
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessor.FilterCloud(cloud, 0.2, 
                                                                                    Eigen::Vector4f{ -5,-6,-3, 1 }, 
                                                                                    Eigen::Vector4f{+28,+7,+1, 1});
    
    //renderPointCloud(viewer, filteredCloud, "inputCloud");

    pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>), roadCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::tie(obstacleCloud, roadCloud) = pointProcessor.segmentPlane(filteredCloud, 30, 0.2);

    renderPointCloud(viewer, roadCloud, "road", Color(0,1,0));
    //renderPointCloud(viewer, obstacleCloud, "obst", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.clusterize(obstacleCloud, 0.8, 3, 3000);

    std::array<Color, 5> colors { Color(1,0,0), Color(1,1,0), Color(0,1,1), Color(1,0,1), Color(0,0,1) };
    for (std::size_t i = 0; i < clusters.size(); ++i)
    {
        renderPointCloud(viewer, clusters[i], std::to_string(i), colors[i % colors.size()]);
        
        //Box boundingBox = pointProcessor.BoundingBox(clusters[i]);
        //renderBox(viewer, boundingBox, i, colors[i % colors.size()], 0.5f);
    }
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
    CameraAngle setAngle = Side; //XY;
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

        viewer->spinOnce();
    } // !viewer->wasStopped ()
}   // main