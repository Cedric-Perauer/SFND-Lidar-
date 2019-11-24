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
		  //using bool, for 2D if value is even even=0 and then x is used, y if otherwise
          //bool even = (depth%2);

		  //for 3D can't use bool, will use normal int
		  int mod = depth%3;
          
		  point[mod] >= (*node)->point[mod] ? helper_insert(&((*node)->right),point,id,depth+1) : helper_insert(&((*node)->left),point,id,depth+1);
		     
		}


	}

	void insert(std::vector<float> point, int id)
	{    
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		helper_insert(&root,point,id,0);
        
	}

	float distance(std::vector<float> a, std::vector<float> b)
	{
		return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2),pow(a[2]-b[2],2));
	}

	void search_helper(std::vector<float> target, Node *node, std::vector<int> &ids, int depth, float Tol) 
	{
		if(node!=NULL)
		{   //see if point is within Threshold
			if((node->point[0]>= (target[0]-Tol)) && (node->point[0] <= (target[0]+Tol)) && (node->point[1]>= (target[1]-Tol)) && (node->point[1] <= (target[1]+Tol))
			&& (node->point[2]>= (target[2]-Tol)) && (node->point[2] <= (target[2]+Tol)))  
			{  
                 float d = distance(std::vector<float> {node->point[0],node->point[1],node->point[2]}, std::vector<float> {target[0],target[1],target[2]});
				 if(d <= Tol)
				 {
					 ids.emplace_back(node->id);
				 }
			}

			if((target[depth%3]-Tol) < node->point[depth%3])
			{
				search_helper(target,node->left,ids,depth+1,Tol);
			}

			if((target[depth%3]+Tol) > node->point[depth%3])
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




