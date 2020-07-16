/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    bool renderScene = false; //true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0); // setCars is cars vector and setGroundSlope is 0. Ground has no slope
    // scan with the lidar
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_rays = lidar->scan();
    // render the lidar_rays
    //renderRays(viewer, lidar->position, lidar_rays);
    // render point clouds
    //renderPointCloud(viewer, lidar_rays,"pcd",Color(1,1,1));
    // TODO:: Create point processor
    // in heap with PCL::PointXYZ
    ProcessPointClouds <pcl::PointXYZ>* processPCD = new ProcessPointClouds<pcl::PointXYZ>(); 
    // in stack
    //ProcessPointClouds <pcl::PointXYZ> processPCD;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr > segmented_cloud = processPCD->SegmentPlane (lidar_rays,100,0.2);
    //auto segmented_cloud  =  processPCD->SegmentPlane (lidar_rays,100,0.2);
    //renderPointCloud(viewer, segmented_cloud.first,"obstacle",Color(1,0,0));
    //renderPointCloud(viewer, segmented_cloud.second,"road",Color(0,1,0));

    // cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPCD->Clustering(segmented_cloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processPCD->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]); 
        Box box = processPCD->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

// for one PCD file
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // TODO:: Create point processor
    ProcessPointClouds <pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-10,-6,-2,1),Eigen::Vector4f(25,7,3,1));
    //renderPointCloud(viewer,filtered_cloud,"filtered_cloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr > segmented_cloud = pointProcessorI->SegmentPlane (filtered_cloud,50,0.2);
    renderPointCloud(viewer, segmented_cloud.first,"obstacle",Color(1,0,0));
    renderPointCloud(viewer, segmented_cloud.second,"road",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmented_cloud.first, 0.6, 10, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+ std::to_string(clusterId),colors[clusterId%3]); 
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId,colors[clusterId%3]);
        ++clusterId;
    }
}

// for stream PCD
void cityBlock_stream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = pointProcessorI.FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-10,-6,-2,1),Eigen::Vector4f(25,7,1,1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr > segmented_cloud = pointProcessorI.SegmentPlane (filtered_cloud,25,0.3); // reduce iteration to 25 because 50 iteration takes time
    renderPointCloud(viewer, segmented_cloud.first,"obstacle",Color(1,0,0));
    renderPointCloud(viewer, segmented_cloud.second,"road",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmented_cloud.first, 0.45, 10, 600);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+ std::to_string(clusterId),colors[clusterId%3]); 
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // for simulated environment
    //simpleHighway(viewer);

    // for single PCD 
    //cityBlock(viewer);
    // previous version
    //while(!viewer->wasStopped()) {
    //    viewer->spinOnce();
    //}

    // for stream PCD
    ProcessPointClouds <pcl::PointXYZI> pointProcessorI; // heap takes more time 
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock_stream(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        
        // repeat 
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}