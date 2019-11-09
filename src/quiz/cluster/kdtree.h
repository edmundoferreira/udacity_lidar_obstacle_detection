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

	void insertNode(Node **node, std::vector<float> point, int id)
	{
		static int depth = 0;
		// std::cout << "current depth = " << depth << std::endl;
		if (*node == NULL) //End of recursion, found an empty node thefore inserting new one in it's place
		{
			*node = new Node(point, id);
		}
		else if (point[depth % 2] < (*node)->point[depth % 2])
		{
			++depth;
			insertNode(&(*node)->left, point, id);
		}
		else
		{
			++depth;
			insertNode(&(*node)->right, point, id);
		}
		depth = 0; //return depth to initial value for next point insertion
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree DONE
		// the function should create a new node and place correctly with in the root
		// std::cout << "inserting new node" << std::endl;
		insertNode(&root, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}


};
