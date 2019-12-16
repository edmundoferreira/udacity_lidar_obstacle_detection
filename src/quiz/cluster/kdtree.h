/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <pcl/common/common.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node *left;
	Node *right;

	Node(PointT p, int setId) : point(p), id(setId), left(NULL), right(NULL) {}
};

template<typename PointT>
struct KdTree
{

public:
	KdTree() : root(NULL) {}

	Node<PointT> *getRootNode()
	{
		return root;
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree DONE
		// the function should create a new node and place correctly with in the root
		// std::cout << "inserting new node" << std::endl;
		insertNode(&root, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		// std::cout << "searching kd-tree" << std::endl;
		searchNode(root, target, distanceTol, ids);
		// std::cout << "found #" << ids.size() << std::endl;
		return ids;
	}

private:
	Node<PointT> *root;

	void insertNode(Node<PointT> **node, PointT point, int id)
	{
		static int depth = 0;
		// std::cout << "current depth = " << depth << std::endl;
		if (*node == NULL) //End of recursion, found an empty node thefore inserting new one in it's place
		{
			*node = new Node<PointT>(point, id);
		}
		else if (point.data[depth % 2] < (*node)->point.data[depth % 2])
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

	void searchNode(Node<PointT> *node, PointT target, float distanceTol, std::vector<int> &ids)
	{
		static int depth = 0;
		if ( node != NULL)
		{
			// std::cout << "current depth = " << depth << std::endl;
			if ((node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) &&
			    (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol))) // point in box
			{
				//calculate if point in circle
				if (std::sqrt(std::pow(node->point.x - target.x, 2) + std::pow(node->point.y - target.y, 2)) <= distanceTol)
				{
					// std::cout << "got point (" << node->point[0] << ", " <<  node->point[1] << ") within distance, #id=" <<  node->id << std::endl;
					ids.push_back(node->id);
				}
			}
			if ((target.data[depth % 2] - distanceTol) < node->point.data[depth % 2]) //left branch
			{
				++depth;
				searchNode(node->left, target, distanceTol, ids);
			}
			if ((target.data[depth % 2] + distanceTol) > node->point.data[depth % 2]) //right branch
			{
				++depth;
				searchNode(node->right, target, distanceTol, ids);
			}
		}
		depth = 0;
	}
};
