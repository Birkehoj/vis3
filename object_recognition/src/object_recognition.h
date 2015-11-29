#ifndef OBJECT_RECOGNITION_H
#define OBJECT_RECOGNITION_H
#include <Eigen/Core>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
// for extract object plane
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include "feature_cloud.h"
#include "object.h"
#include "object_recognition_types.h"
using std::vector;
class Object_recognition
{
public:  
  struct Result
  {
    Result()
      : transformed_cloud(new CloudT), object_name("Unknown") {}
    std::string object_name;
    float score;
    CloudT::Ptr transformed_cloud;
    Transformation transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  const vector<Object>& getObjects() const {
    return objects;
  }

  Object_recognition(const std::string object_folder_path);
  void getObjects(const CloudT::Ptr& scene, std::vector<Result>& object_poses);
  void showPointCloud(const CloudT::Ptr& cloud, char* name);
  void displayAlignment(const CloudT::Ptr& scene, const vector<Result> &results);
  void displayAlignment(const CloudT::Ptr &scene, const CloudT::Ptr &aligned_object);
  void displaySegments(const CloudT::Ptr& scene);
  void segmentSceneInObjects(const CloudT::Ptr& cloud, std::vector<CloudT::Ptr> &scene_objects);
  void setObjects();
private:
  bool getObjectPointsOnPlane(const CloudT::Ptr &scene, CloudT::Ptr &objects_pose);
  void getSceneSegmentFeatures(const std::vector<CloudT::Ptr> &scene_clouds, std::vector<Feature_cloud>& scene_objects);
  void alignment(const std::vector<Feature_cloud>& scene_objects, std::vector<Result>& objects_pose);
  void refineAlignment(const CloudT::Ptr &query, Result &res);
  std::string object_folder_path;
  std::vector<Object> objects;
  double max_z;
};

#endif // OBJECT_RECOGNITION_H
