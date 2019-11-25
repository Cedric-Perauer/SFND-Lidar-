/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZ point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZ arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
    void helper_insert(Node **node, pcl::PointXYZ cloud_point, int id,int depth){
		
		//no root => Tree is completely empty 
		if(*node == NULL) 
		{
			*node = new Node(cloud_point,id);

		}
		else
		{
		  //using bool, for 2D if value is even even=0 and then x is used, y if otherwise
          //bool even = (depth%2);

		  //for 3D can't use bool, will use normal int
		  int mod = depth%3;
          
		  if(mod == 0){
              cloud_point.x >= (*node)->point.x ? helper_insert(&((*node)->right),cloud_point,id,depth+1) : helper_insert(&((*node)->left),cloud_point,id,depth+1);
		  }

		  else if(mod ==1) 
		  {
			  cloud_point.y >= (*node)->point.y ? helper_insert(&((*node)->right),cloud_point,id,depth+1) : helper_insert(&((*node)->left),cloud_point,id,depth+1);
		  }

		  else {
			  cloud_point.z >= (*node)->point.z ? helper_insert(&((*node)->right),cloud_point,id,depth+1) : helper_insert(&((*node)->left),cloud_point,id,depth+1);
		  }
		     
		}
	}

	void insert(pcl::PointXYZ cloud_point, int id)
	{    
		// TODO: Fill in this function to insert a new cloud_point into the tree
		// the function should create a new node and place correctly with in the root 
		
		helper_insert(&root,cloud_point,id,0);
        
	}

	

	void search_helper(pcl::PointXYZ cloud_point, Node *node, std::vector<int> &ids, int depth, float Tol) 
	{
		if(node!=NULL)
		{   //see if cloud_point is within Threshold
			if((node->point.x>= (cloud_point.x-Tol)) && (node->point.x <= (cloud_point.x+Tol)) && (node->point.y>= (cloud_point.y-Tol)) && (node->point.y <= (cloud_point.y+Tol))
			&& (node->point.z>= (cloud_point.z-Tol)) && (node->point.z <= (cloud_point.z+Tol)))  
			{  
                 float d = sqrt((node->point.x-cloud_point.x)+pow(node->point.y-cloud_point.y,2)+pow(node->point.z-cloud_point.z,2));
				 if(d <= Tol)
				 {
					 ids.push_back(node->id);
				 }
			}
			
	if(depth%3 == 0){
        if((cloud_point.x - Tol) < node->point.x) search_helper(cloud_point, node->left,ids, depth+1, Tol);
        if((cloud_point.x + Tol) > node->point.x) search_helper(cloud_point, node->right, ids, depth+1, Tol);
      }
    else if(depth%3 == 1)
	{
        if((cloud_point.y - Tol) < node->point.y) search_helper(cloud_point, node->left, ids, depth+1, Tol);
        if((cloud_point.y + Tol) > node->point.y) search_helper(cloud_point, node->right, ids, depth+1, Tol);
      }

	  else 
	  {
        if((cloud_point.z - Tol) < node->point.z) search_helper(cloud_point, node->left, ids, depth+1, Tol);
        if((cloud_point.z + Tol) > node->point.z) search_helper(cloud_point, node->right, ids, depth+1, Tol);
      }
		
}

	}




	// return a list of cloud_point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZ target, float Tol)
	{   
		std::vector<int> ids;
		search_helper(target, root, ids, 0, Tol);
		return ids;
	}
	

};




