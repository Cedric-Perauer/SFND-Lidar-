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
    bool renderScene = false; //show scene of the street and cars or not
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar_obj = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar_obj->scan();
    //renderPointCloud(viewer,cloud,"Input Cloud");
  
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> Point_Processor;  
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = Point_Processor.SegmentPlane(inputCloud, 100, 0.2); //pcl Function for testing
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = Point_Processor.RANSAC_Segmentation(inputCloud,100,0.19); 
 
       

    
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
     

    KdTree<pcl::PointXYZ>* tree = new KdTree<pcl::PointXYZ>;
  
    for (int i=0; i<segmentCloud.first->points.size(); i++) 
    	tree->insert(segmentCloud.first->points[i],i,3); 
    

    
   //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = Point_Processor.Clustering(segmentCloud.first,1.0,3,30);

   //TO-DO : own EC algorithm 
   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = Point_Processor.euclideanCluster(segmentCloud.first, tree, 1.01,3, 30,3);

    int clusterId = 0; 
    std::vector<Color> colors = {Color(0,0,1),Color(1,0,1),Color(1,1,0)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudClusters)  //loop through clusters
    {
        std::cout << "Cluster Size ";        
        Point_Processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        //render BB
        Box box = Point_Processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // Experiment with the ? values and find what works best  
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud; 
  filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (30, 7, 2, 1),Eigen::Vector4f (-2.0, -1.5, -2, 1), Eigen::Vector4f ( 2.7, 1.5, 0, 1));
 

    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2); //pcl Function for testing
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RANSAC_Segmentation(filterCloud,100,0.2); 

    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
     
    KdTree<pcl::PointXYZI>* tree = new KdTree<pcl::PointXYZI>;
  
    for (int i=0; i<segmentCloud.first->points.size(); i++) 
    	tree->insert(segmentCloud.first->points[i],i,3); 
    

    
   //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first,0.5,10,250);

   //TO-DO : own EC algorithm 
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, 0.5,10, 200,3);

    int clusterId = 0; 
    std::vector<Color> colors = {Color(0,0,1),Color(1,0,1),Color(1,1,0)};
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster:cloudClusters)  //loop through clusters
    {
        std::cout << "Cluster Size ";        
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        //render BB
        Box box = pointProcessorI->BoundingBox(cluster);
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while(!viewer->wasStopped())
    {
        //Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes(); 

      // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
}

    //simpleHighway(viewer);
    
}
