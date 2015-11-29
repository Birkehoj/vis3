#ifndef OBJECT_RECOGNITION_TYPES
#define OBJECT_RECOGNITION_TYPES
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <iostream>
using std::cout;
using std::endl;
using std::vector;
using std::string;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
typedef Eigen::Matrix4f Transformation;
#endif // OBJECT_RECOGNITION_TYPES

