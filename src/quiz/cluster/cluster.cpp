/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 0, 0, 0, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node<pcl::PointXYZ>* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point.x, window.y_min, 0),pcl::PointXYZ(node->point.x, window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point.x;
			upperWindow.x_min = node->point.x;
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point.y, 0),pcl::PointXYZ(window.x_max, node->point.y, 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point.y;
			upperWindow.y_min = node->point.y;
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void Proximity(int index, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<bool> &pointsMarked, std::vector<int> &cluster, KdTree<pcl::PointXYZ> *tree, float distanceTol)
{
	// mark point as processed
	// add point to cluster
	// nearby points = tree(point)
	// Iterate through each nearby point
	// 	If point has not been processed
	// 		Proximit
	pointsMarked[index] = true;
	cluster.push_back(index);
	auto nearbyIndexVec = tree->search(cloud->points[index], distanceTol);
	for (auto nearIndex : nearbyIndexVec)
		if (!pointsMarked[nearIndex])
			Proximity(nearIndex, cloud, pointsMarked, cluster, tree, distanceTol);
}

std::vector<std::vector<int>> euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, KdTree<pcl::PointXYZ> *tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	//    list of clusters
	//     Iterate through each point
	//         If point has not been processed
	//             Create cluster
	//             Proximity(point, cluster)
	//             cluster add clusters
	//     return clusters
	std::vector<std::vector<int>> clusters;

	std::vector<bool> pointsMarked(cloud->size(), false);
	//iterate over all points in vector of x,y
	for (auto i = 0; i < cloud->size(); ++i)
	{
		if (!pointsMarked[i])
		{
			std::vector<int> cluster;
			Proximity(i, cloud, pointsMarked, cluster, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}

	return clusters;
}

int main ()
{

	// Create viewer
	Box window;
	window.x_min = -10;
	window.x_max =  10;
	window.y_min = -10;
	window.y_max =  10;
	window.z_min =  0;
	window.z_max =  0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = {{-6.2, 7}, {-6.3, 8.4}, {-5.2, 7.1}, {-5.7, 6.3}, {7.2, 6.1}, {8.0, 5.3}, {7.2, 7.1}, {0.2, -7.1}, {1.7, -6.9}, {-1.2, -7.2}, {2.2, -8.9}};
	// std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	// for ( auto point : *cloud)
	// 	std::cout << "p(" << point.data[0] << "," << point.y << ")" << std::endl;


	KdTree<pcl::PointXYZ> *tree = new KdTree<pcl::PointXYZ>;

	// for (int i = 0; i < points.size(); i++)
	// 	tree->insert(points[i], i);

	int i = 0;
	for ( auto point : *cloud)
	{
		tree->insert(point, i);
		++i;
	}


	int it = 0;
	render2DTree(tree->getRootNode(), viewer, window, it);

	std::cout << "Test Search" << std::endl;
	std::vector<int> nearby = tree->search(pcl::PointXYZ({-6, 7}), 3.0);
	for (int index : nearby)
		std::cout << index << ",";
	std::cout << std::endl;

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	//
	std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, 3.0);
	//
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Render clusters
	int clusterId = 0;
	std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(1, 1, 0)};
	for (std::vector<int> cluster : clusters)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
		for (int indice : cluster)
			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0], points[indice][1], 0));
		renderPointCloud(viewer, clusterCloud, "cluster" + std::to_string(clusterId), colors[clusterId % 3]);
		++clusterId;
	}
	if (clusters.size() == 0)
		renderPointCloud(viewer, cloud, "data");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
