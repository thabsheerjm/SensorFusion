// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <filesystem>

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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid filtering
    typename pcl::PointCloud<PointT>::Ptr cloudVGFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudVGFiltered);

    // Region of Interest (ROI) filtering
    typename pcl::PointCloud<PointT>::Ptr cloudBoxFiltered(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudVGFiltered);
    cropBoxFilter.filter(*cloudBoxFiltered);

    // Rooftop point removal
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(cloudBoxFiltered);

    std::vector<int> indices;
    roofFilter.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices = std::move(indices);

    // Extract non-rooftop points
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract(true);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(cloudBoxFiltered);
    extract.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    std::cout << "filtering took " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << " milliseconds" << std::endl;

    return cloudFiltered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>()};

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract; 

    extract.setInputCloud(cloud); 
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    pcl::PointIndices::Ptr bestInliers(new pcl::PointIndices());

    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        // Randomly select three distinct points from the cloud
        PointT point1 = cloud->points[rand() % cloud->points.size()];
        PointT point2 = cloud->points[rand() % cloud->points.size()];
        PointT point3 = cloud->points[rand() % cloud->points.size()];

        // Compute the plane coefficients (Ax + By + Cz + D = 0)
        float A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        float B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        float C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        float D = -(A * point1.x + B * point1.y + C * point1.z);

        // Normalize the coefficients to improve numerical stability
        float norm = sqrt(A * A + B * B + C * C);
        A /= norm;
        B /= norm;
        C /= norm;
        D /= norm;

        pcl::PointIndices::Ptr currentInliers(new pcl::PointIndices());

        // Measure distance between every point and the plane
        for (size_t idx = 0; idx < cloud->points.size(); ++idx)
        {
            PointT currentPoint = cloud->points[idx];
            float distance = fabs(A * currentPoint.x + B * currentPoint.y + C * currentPoint.z + D);

            if (distance <= distanceThreshold)
            {
                currentInliers->indices.push_back(idx);
            }
        }

        // Update best inliers if current iteration is better
        if (currentInliers->indices.size() > bestInliers->indices.size())
        {
            bestInliers = currentInliers;
        }
    }

    if (bestInliers->indices.empty())
    {
        std::cerr << "Failed to detect a plane." << std::endl;
    }

    return SeparateClouds(bestInliers, cloud);
}



template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& processedPoints, int index, typename pcl::PointCloud<PointT>::Ptr cluster, KdTree* tree, float clusterTolerance)
{
	processedPoints[index] = true; 
	cluster->push_back(cloud->points[index]);

    PointT point = cloud->points[index]; 
	std::vector<int> proximity = tree->search({point.x, point.y, point.z}, clusterTolerance);

	for (int id : proximity )
	{
		if (!processedPoints[id])
		{
			clusterHelper(cloud, processedPoints, id, cluster, tree, clusterTolerance);
		}
	}
}




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringEuclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	KdTree* tree = new KdTree; 
    for (int i =0; i< cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        tree->insert({point.x, point.y, point.z}, i);
    }

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processedPoints(cloud->points.size(), false); 

	for (int i=0; i<cloud->points.size();++i)
	{
		if (processedPoints[i])
		continue; 

		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>); 
        clusterHelper(cloud, processedPoints, i , cluster, tree, clusterTolerance); 

        if ((cluster->size() >= minSize) && (cluster->size()<= maxSize))
        {
            cluster->width = cluster->size(); 
            cluster->height = 1; 
            cluster->is_dense = true; 
            clusters.push_back(cluster);
        }
	}
 
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
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});
    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());
    return paths;

}