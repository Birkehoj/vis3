#ifndef FEATURE_CLOUD_H
#define FEATURE_CLOUD_H
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include "object_recognition_types.h"
class Feature_cloud
{
public:
  // A bit of shorthand
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> CloudT;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormalT;
  //typedef feature::ECSAD<PointT>::Histogram FeatureT;
  //typedef feature::DistanceNormalHistogram<PointT,8,16>::Histogram FeatureT;
  //typedef pcl::SHOT352 FeatureT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > SearchMethodT;

  Feature_cloud(CloudT::Ptr xyz, std::string);

  // Get a pointer to the cloud of feature descriptors
  FeatureCloudT::Ptr getLocalFeatures () const {
    return features_;
  }
  CloudT::Ptr getCloud() const {
    return xyz_;
  }
  std::string name;
protected:
  Feature_cloud(std::string name="Unknown obj");
  SearchMethodT::Ptr search_method_;
  CloudT::Ptr xyz_; // downsampled cloud
  double normal_radius_;
  double feature_radius_;
  double resolution_;
  FeatureCloudT::Ptr features_;
  void computeFeatures(CloudT::Ptr xyz);
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
};
#endif // FEATURE_CLOUD_H
