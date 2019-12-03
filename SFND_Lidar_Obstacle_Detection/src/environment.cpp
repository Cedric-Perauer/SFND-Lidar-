/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <typeinfo>




void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, boost::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud; 
  filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (30, 7, 2, 1),Eigen::Vector4f (-2.0, -1.5, -2, 1), Eigen::Vector4f ( 2.7, 1.5, 0, 1));
 

    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2); //pcl Function for testing
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RANSAC_Segmentation(filterCloud,100,0.2); 

    
     
    boost::shared_ptr<KdTree<pcl::PointXYZI>> tree = boost::make_shared<KdTree<pcl::PointXYZI>>();
    


    for (int i=0; i<segmentCloud.first->points.size(); i++) 
    	tree->insert(segmentCloud.first->points[i],i,3); 
    
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
   //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first,0.5,10,250);

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
    auto pointProcessorI = boost::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
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
}
