/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>
#include "bits/stdc++.h"


template<typename PointT>
// Structure to represent node of kd tree
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node(){

	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(nullptr)
	{}
    
	void helper_insert(Node<PointT>** node,  PointT cloud_point, int id, int depth,int dim)
    { 
	  //no root => Tree is completely empty 
      if(*node == nullptr)
	  { 
		  
		(*node) = boost::make_shared<Node<PointT>>(cloud_point, id);

	  }
      else
      {
		  //using bool, for 2D if value is even even=0 and then x is used, y if otherwise
          //bool even = (depth%2);
        int mod = depth % dim;
        if(mod == 0){
			 cloud_point.x >= (*node)->point.x ? helper_insert(&((*node)->right),cloud_point,id,depth+1,dim) : helper_insert(&((*node)->left),cloud_point,id,depth+1,dim);

        }
        else if(mod==1){
				  cloud_point.y >= (*node)->point.y ? helper_insert(&((*node)->right),cloud_point,id,depth+1,dim) : helper_insert(&((*node)->left),cloud_point,id,depth+1,dim);
        
        }

		else{
			cloud_point.z >= (*node)->point.z ? helper_insert(&((*node)->right),cloud_point,id,depth+1,dim) : helper_insert(&((*node)->left),cloud_point,id,depth+1,dim);
        }
        
      }
	  
    }
  
	void insert(PointT cloud_point, int id,int dim)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		helper_insert(&root, cloud_point, id, 0,dim);
	}

void searchHelper(PointT cloud_point, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids,int dim)
{
	if(node != NULL)
    {
	  if((node->point.x >= (cloud_point.x-distanceTol)&&(node->point.x <= (cloud_point.x+distanceTol)))&&(node->point.y >= (cloud_point.y-distanceTol)&&(node->point.y <= (cloud_point.y+distanceTol)))
	  &&(node->point.z >= (cloud_point.z-distanceTol)&&(node->point.z <= (cloud_point.z+distanceTol))))
      {
        float distance = sqrt(pow((node->point.x - cloud_point.x),2)  + pow((node->point.y - cloud_point.y),2)  + pow((node->point.z - cloud_point.z),2));
        if(distance <= distanceTol) ids.push_back(node->id);
      }
	  int mod = depth%dim; 
      if(mod == 0){
        if((cloud_point.x - distanceTol) < node->point.x) searchHelper(cloud_point, node->left, depth+1, distanceTol, ids,dim);
        if((cloud_point.x + distanceTol) > node->point.x) searchHelper(cloud_point, node->right, depth+1, distanceTol, ids,dim);
      }
      else if(mod==1){
        if((cloud_point.y - distanceTol) < node->point.y) searchHelper(cloud_point, node->left, depth+1, distanceTol, ids,dim);
        if((cloud_point.y + distanceTol) > node->point.y) searchHelper(cloud_point, node->right, depth+1, distanceTol, ids,dim);
      }

	  else{
        if((cloud_point.z - distanceTol) < node->point.z) searchHelper(cloud_point, node->left, depth+1, distanceTol, ids,dim);
        if((cloud_point.z + distanceTol) > node->point.z) searchHelper(cloud_point, node->right, depth+1, distanceTol, ids,dim);
      }
      
    }
}
  
	// return a list of point ids in the tree that are within distance of cloud_point
	std::vector<int> search(PointT cloud_point, float distanceTol,int dim)
	{
		std::vector<int> ids;
    searchHelper(cloud_point, root, 0, distanceTol, ids,dim);
		return ids;
	}
	

};


