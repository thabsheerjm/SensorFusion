//run lidar obstacle detection
#include "utils/sensors/lidar.h"
#include "utils/processPointClouds/processPointClouds.h"
#include "utils/processPointClouds/processPointClouds.cpp"
# include <vtkObject.h>
#include <filesystem>
#include "utils/render/render.h"
#include "utils/render/environment.h"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.18, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 4, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneRansac(filteredCloud, 100, 0.3);
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0)); 
    renderPointCloud(viewer, segmentCloud.second,"planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringEuclidean(segmentCloud.first, 0.5, 10, 500);

    int clusterId = 0 ;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(1,1,0)};

    for (size_t i = 0; i < cloudClusters.size(); ++i)
    {
        const auto& cluster = cloudClusters[i];

        std::cout << "Cluster " << i << " has a size of ";
        pointProcessorI->numPoints(cluster);

        std::string cloudName = "obstacleCloud" + std::to_string(i);
        renderPointCloud(viewer, cluster, cloudName, colors[i % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, i);
    }
}


int main (int argc, char** argv)
{
    vtkObject::GlobalWarningDisplayOff(); //supress warnings
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../data/pcd/data_1"); 
    auto streamIterator = stream.begin(); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ()) 
    {
        viewer->removeAllPointClouds(); 
        viewer->removeAllShapes(); 

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator =  stream.begin();

        
        viewer->spinOnce (100);
    } 
}