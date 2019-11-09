/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree
{

public:
	KdTree() : root(NULL) {}

	Node *getRootNode()
	{
		return root;
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
		// std::cout << "searching kd-tree" << std::endl;
		searchNode(root, target, distanceTol, ids);
		// std::cout << "found #" << ids.size() << std::endl;
		return ids;
	}

private:
	Node *root;

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

	void searchNode(Node *node, std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		static int depth = 0;
		if ( node != NULL)
		{
			// std::cout << "current depth = " << depth << std::endl;
			if ((node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) &&
			    (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))) // point in box
			{
				//calculate if point in circle
				if (std::sqrt(std::pow(node->point[0] - target[0], 2) + std::pow(node->point[1] - target[1], 2)) <= distanceTol)
				{
					// std::cout << "got point (" << node->point[0] << ", " <<  node->point[1] << ") within distance, #id=" <<  node->id << std::endl;
					ids.push_back(node->id);
				}
			}
			if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) //left branch
			{
				++depth;
				searchNode(node->left, target, distanceTol, ids);
			}
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) //right branch
			{
				++depth;
				searchNode(node->right, target, distanceTol, ids);
			}
		}
		depth = 0;
	}
};
