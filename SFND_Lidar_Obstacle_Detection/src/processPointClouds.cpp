// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint,Eigen::Vector4f roof_minPoint, Eigen::Vector4f roof_maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    //Voxel Filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor; 
    sor.setInputCloud(cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes); //leaf size
    sor.filter (*cloud_filtered);
    
    //Crop Based on ROI 
    typename pcl::PointCloud<PointT>::Ptr cloud_roi (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region (true); //create Crop, set to true so we can extract the indices of points being removed

    region.setMax(maxPoint);
    region.setMin(minPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_roi);

    //Crop Based on Roof position   
    std::vector<int> indices; //indices for roof result
    pcl::CropBox<PointT> roof (true); //create Crop, set to true so we can extract the indices of points being removed

    roof.setMax(roof_maxPoint);
    roof.setMin(roof_minPoint);
    roof.setInputCloud(cloud_roi);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); //create indices 
    for(int point_idx:indices){
        inliers->indices.push_back(point_idx);
    }

    pcl::ExtractIndices<PointT> extract; //generate obstacle cloud
    extract.setInputCloud (cloud_roi);
    extract.setIndices (inliers);   
    extract.setNegative (true);  //set it to negative, all inliers are removed
    extract.filter (*cloud_roi);  //perfom the filtering 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return cloud_roi;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  //create obstacle and ground cloud
     typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT> ());
     typename pcl::PointCloud<PointT>::Ptr ground_cloud (new pcl::PointCloud<PointT> ());
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    for(int index : inliers->indices ) { //iterate over all inliers and grab index  
        ground_cloud->points.push_back(cloud->points[index]);    //add points to ground cloud
    }
    pcl::ExtractIndices<PointT> extract; //generate obstacle cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);   
    extract.setNegative (true);  //set it to negative, all inliers are removed
    extract.filter (*obstacle_cloud);  //perfom the filtering 
 
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, ground_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
  // Optional
    seg.setOptimizeCoefficients (true);
  // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);  //use RANSAC
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size()==0) {
        std::cout << "Could not estimate the ground plane for this data" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);   //create KD Tree for storage of the points
    tree->setInputCloud(cloud); //set input cloud
    std::vector<pcl::PointIndices> cluster_ind; 
    //set ec params
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_ind);
    //loop through clusters
    for(pcl::PointIndices getIndicies:cluster_ind) 
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);   //create new PCL for each cluster
        
        for(int index : getIndicies.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 

        clusters.push_back(cloudCluster);

}

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

//my own Functions

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{   
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


		float a,b,c,d,distance;
		a = h; 
		b = j; 
		c = k; 
		d = -(h*x1 + j*y1 + k*z1);
        

		for(int i = 0 ; i < cloud->points.size() ; i++) {

			if(inliers.count(i)>0){
				continue;
			}

			PointT point= cloud->points[i];
			float px,py,pz,distance;  //point coordiantes and distance measurement 
			px = point.x;
			py = point.y; 
            pz = point.z;

			distance = fabs(a*px+b*py+c*pz+d)/(sqrt(a*a+b*b+c*c));
			
			if(distance<=distanceTol) {
				inliers.insert(i);   //add to inliers if under distance threshold
			}
			 //std::cout << "Iteration :" << i << " with : " << inliers.size() << " points " << std::endl;
		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;                //if 
		}
  
	}

	return inliersResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC_Segmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{   auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceTol);
    
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> {cloudOutliers,cloudInliers};

}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int i,typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed,boost::shared_ptr<KdTree<PointT>> tree,float distanceTol,int dim)
{
  processed[i] = true; //mark point as processed
  cluster.emplace_back(i);  //add index to cluster
  std::vector<int> nearby = tree->search(cloud->points[i],distanceTol,dim); // return a list of point ids in the tree that are within distance of target

  //iterate through nearby points
  for(int idx : nearby)
  {
	  if(!processed[idx])
	     Proximity(idx,cloud,cluster,processed,tree, distanceTol,dim);
  }
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, boost::shared_ptr<KdTree<PointT>> tree, float distanceTol, int minSize, int maxSize,int dim)

{   auto startTime = std::chrono::steady_clock::now();

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(),false);  //track if point has been processed or not, initalize as false

	for(size_t i =0; i < cloud->points.size(); i++) //loop through number of points
	{   
		if(!processed[i])
		{
			std::vector<int> idx; //holds cluster cloud indeces 
            typename pcl::PointCloud<PointT>::Ptr Cluster (new pcl::PointCloud<PointT>);
            Proximity(i,cloud,idx,processed,tree,distanceTol,dim);
            
            if(idx.size()>=minSize && idx.size() <= maxSize)  //make sure cluster is within size boundaries
            {   
                for(int j = 0; j <idx.size();j++)  //loop through points of cluster
                {
                    PointT point = cloud->points[idx[j]]; //pick out point from cluster
                    Cluster->push_back(point);
                    
                }         

                Cluster->width = Cluster->points.size();
                Cluster->height = 1; 
                Cluster->is_dense = true;        
                clusters.push_back(Cluster); //add cluster to Clusters list 
            }

            else
            {
                for(int j = 1; j <idx.size();j++)  //loop through points of cluster
                {
                    processed[idx[j]] = false; 
                }     
            }

        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Euclidean Clustering took " << elapsedTime.count() << " milliseconds" << std::endl;
    
	return clusters;
}