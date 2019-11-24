/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
    void helper_insert(Node **node, std::vector<float> point, int id,int depth){
		
		//no root => Tree is completely empty 
		if(*node == NULL) 
		{
			*node = new Node(point,id);

		}
		else
		{
          bool even = (depth%2); //using bool, if value is even even=0 and then x is used, y if otherwise
          
		  point[even] >= (*node)->point[even] ? helper_insert(&((*node)->right),point,id,depth+1) : helper_insert(&((*node)->left),point,id,depth+1);
		     
		}


	}

	void insert(std::vector<float> point, int id)
	{    
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		helper_insert(&root,point,id,0);
        
	}

	float distance(std::pair<float,float> a, std::pair<float,float> b)
	{
		return sqrt(pow(a.first-b.first,2)+pow(a.second-b.second,2));
	}

	void search_helper(std::vector<float> target, Node *node, std::vector<int> &ids, int depth, float Tol) 
	{
		if(node!=NULL)
		{   //see if point is within Threshold
			if((node->point[0]>= (target[0]-Tol)) && (node->point[0] <= (target[0]+Tol)) && (node->point[1]>= (target[1]-Tol)) && (node->point[1] <= (target[1]+Tol)))  
			{  
                 float d = distance(std::pair<float,float> {node->point[0],node->point[1]}, std::pair<float,float> {target[0],target[1]});
				 if(d <= Tol)
				 {
					 ids.emplace_back(node->id);
				 }
			}

			if((target[depth%2]-Tol) < node->point[depth%2])
			{
				search_helper(target,node->left,ids,depth+1,Tol);
			}

			if((target[depth%2]+Tol) > node->point[depth%2])
			{
				search_helper(target,node->right,ids,depth+1,Tol);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{   
		std::vector<int> ids;
		search_helper(target, root, ids, 0, distanceTol);
		return ids;
	}
	

};




