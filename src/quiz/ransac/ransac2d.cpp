/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

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

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersTemp;
	srand(time(NULL));

	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	// TODO: Fill in this function DONE

	// For max iterations
	for(auto iteration= 0; iteration<maxIterations; iteration++) {

		// Randomly sample subset and fit line
		auto index1 = rand() % cloud->points.size();
		auto index2 = rand() % cloud->points.size();
		if (index1 != index2) //escape equal indexes ie single point line will fail
		{
			inliersTemp.clear();
			inliersTemp.insert(index1);
			inliersTemp.insert(index2);
			// std::cout << "#Iteration " << iteration << "(p" << index1 << ",p" << index2 << ")" << std::endl;
			//for p1=(x1,y1) and p2=(x2,y2) => A=y1-y2 B=x2-x1  C=x1*y2-x2*y1
			p1 = cloud->points[index1];
			p2 = cloud->points[index2];
			auto A = p1.y - p2.y;
			auto B = p2.x - p1.x;
			auto C = p1.x * p2.y - p2.x * p1.y;

			// Measure distance between every point and fitted line
			for (auto index = 0; index < cloud->points.size(); index++)
			{
				//escape p1 p2 that are inliers already
				if(inliersTemp.count(index)>0)
					continue;
				// If distance is smaller than threshold count it as inlier
				// d=∣Ax+By+C∣/sqrt(A^2 +B^2 )
				auto d = std::abs(A*cloud->points[index].x + B*cloud->points[index].y + C) / std::sqrt(std::pow(A, 2) + std::pow(B, 2));
				if (d < distanceTol)
				{
					inliersTemp.insert(index);
					// std::cout << "index=" << index << " is an inlier" << std::endl;
				}
			}
			// std::cout << "#LocalMaxInliers " << inliersTemp.size() << std::endl;
			// Return indicies of inliers from fitted line with most inliers
			if(inliersTemp.size()>inliersResult.size())
				inliersResult = inliersTemp;
			// std::cout << "#GlobalMaxInliers " << inliersResult.size() << std::endl;

		}else{
			iteration--;
		}

	}

	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersTemp;
	srand(time(NULL));

	Eigen::Vector3f p1;
	Eigen::Vector3f p2;
	Eigen::Vector3f p3;
	// TODO: Fill in this function DONE

	// For max iterations
	for(auto iteration= 0; iteration<maxIterations; iteration++) {

		// Randomly sample subset and fit line
		auto index1 = rand() % cloud->points.size();
		auto index2 = rand() % cloud->points.size();
		auto index3 = rand() % cloud->points.size();
		if (index1 != index2 && index1 != index3 && index2 != index3) //escape equal indexes ie single point line will fail
		{
			inliersTemp.clear();
			inliersTemp.insert(index1);
			inliersTemp.insert(index2);
			inliersTemp.insert(index3);
			p1 = cloud->points[index1].getVector3fMap();
			p2 = cloud->points[index2].getVector3fMap();
			p3 = cloud->points[index3].getVector3fMap();

			auto v1 = p2 - p1;
			auto v2 = p3 - p1;

			auto vNormal= v1.cross(v2);

			auto A = vNormal.x();
			auto B = vNormal.y();
			auto C = vNormal.z();
			auto D = -(vNormal.x()*p1.x() + vNormal.y()*p1.y() + vNormal.z()*p1.z());

			// Measure distance between every point and fitted line
			for (auto index = 0; index < cloud->points.size(); index++) {
				//escape p1 p2 that are inliers already
				if(inliersTemp.count(index)>0)
					continue;
				// If distance is smaller than threshold count it as inlier
				// d = |A*x+B*y+C*z+D|/sqrt(A^2+B^2+C^2).d=∣A∗x+B∗y+C∗z+D∣/sqrt(A^2 +B^2 +C^2)
				auto d = std::abs(A*cloud->points[index].x + B*cloud->points[index].y + C*cloud->points[index].z + D) / std::sqrt(std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2));
				if (d < distanceTol)
				{
					inliersTemp.insert(index);
				}
			}
			// Return indicies of inliers from fitted line with most inliers
			if(inliersTemp.size()>inliersResult.size())
				inliersResult = inliersTemp;

		}else{
			iteration--;
		}

	}

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function DONE
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

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
