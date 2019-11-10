// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>


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

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time filtering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering DONE
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>());

    // create crop box
    typename pcl::CropBox<PointT> boxFilter;
    boxFilter.setInputCloud(cloud);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.filter(*cloud_cropped);

    typename pcl::CropBox<PointT> selfCarCrop;
    std::vector<int> indices;
    selfCarCrop.setInputCloud(cloud_cropped);
    selfCarCrop.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    selfCarCrop.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    selfCarCrop.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (auto point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);

    // Note seems to be faster to crop first then voxel filter
    // Create the filtering object
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_cropped);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane DONE
    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_obst(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(*cloud_plane);


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_obst);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud. DONE
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles DONE
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->height = 1;
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliers;
	std::unordered_set<int> inliersTemp;
	srand(time(NULL));

	Eigen::Vector3f p1;
	Eigen::Vector3f p2;
	Eigen::Vector3f p3;
	// TODO: Fill in this function DONE

	// For max iterations
    for (auto iteration = 0; iteration < maxIterations; iteration++)
    {

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
				if (d < distanceThreshold)
				{
					inliersTemp.insert(index);
				}
			}
			// Return indicies of inliers from fitted line with most inliers
			if(inliersTemp.size()>inliers.size())
				inliers = inliersTemp;

		}else{
			iteration--;
		}

	}

    typename pcl::PointCloud<PointT>::Ptr cloudPlane(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudObst(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudPlane->points.push_back(point);
		else
			cloudObst->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudObst, cloudPlane);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}



