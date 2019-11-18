/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <bits/stdc++.h>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

/*
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;   //hold best inliers (most inliers)
	srand(time(NULL));
	
	// TODO: Fill in this function
     
	// For max iterations 
    while(maxIterations--) {

		//randomly pick two points
		std::unordered_set<int> inliers;   //holds index for point of cloud 
		while(inliers.size() <2)
		    inliers.insert(rand()%(cloud->points.size()));    //insert random points, modulu returns between 0 and points.size() from cloud  
	    float x1,x2, y1, y2;  //start and end point

		auto itr = inliers.begin(); //pointer to beginning of inliers
        
		//dereference pointer itr to get value of index
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		++itr;
        x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		
		float a,b,c; 
		a = y1-y2;
		b = x2-x1; 
		c = (x1*y2-x2*y1);
        

		for(int i = 0 ; i < cloud->points.size() ; i++) {

			if(inliers.count(i)>0){
				continue;
			}

			pcl::PointXYZ point= cloud->points[i];
			float px,py,d;  //point coordiantes and distance measurement 
			px = point.x;
			py = point.y; 
            
			d = fabs(a*px+b*py+c)/(sqrt(a*a+b*b));
			
			if(d<=distanceTol) {
				inliers.insert(i);   //add to inliers if under distance threshold
			}
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;                //if 
		}

	} 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

*/

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{   
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;   //hold best inliers (most inliers)
	srand(time(NULL));
	
	// TODO: Fill in this function
     
	// For max iterations 
    while(maxIterations--) {

		//randomly pick two points
		std::unordered_set<int> inliers;   //holds index for point of cloud 
		while(inliers.size() <3)     //3D now 
		    inliers.insert(rand()%(cloud->points.size()));    //insert random points, modulu returns between 0 and points.size() from cloud  
	    float x1,x2,x3, y1, y2,y3,z1,z2,z3;  //start and end point

		auto itr = inliers.begin(); //pointer to beginning of inliers
        
		//dereference pointer itr to get value of index
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		++itr;
        x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
		++itr; 
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
		
		//based on v1xv2 
        float h,j,k; 
		h = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		j = (z2-z1) *(x3-x1) - (x2-x1) * (z3-z1); 
        k = (x2-x1) *(y3-y1) - (y2-y1) * (x3-x1);

´    
		float a,b,c,d,distance;
		a = h; 
		b = j; 
		c = k; 
		d = -(h*x1 + j*y1 + k*z1);
        

		for(int i = 0 ; i < cloud->points.size() ; i++) {

			if(inliers.count(i)>0){
				continue;
			}

			pcl::PointXYZ point= cloud->points[i];
			float px,py,pz,distance;  //point coordiantes and distance measurement 
			px = point.x;
			py = point.y; 
            pz = point.z;

			distance = fabs(a*px+b*py+c*pz+d)/(sqrt(a*a+b*b+c*c));
			
			if(distance<=distanceTol) {
				inliers.insert(i);   //add to inliers if under distance threshold
			}
			 std::cout << "Iteration :" << i << " with : " << inliers.size() << " points " << std::endl;
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;                //if 
		}
  
	} 
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC for plane segementation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
