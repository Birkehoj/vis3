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
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <feature_cloud.h>

using std::vector;
class Object_recognition
{
public:
  // A bit of shorthand
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  // A struct for storing alignment results
  struct Result
  {
    float score;
    Eigen::Matrix4f transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  vector<Feature_cloud> getObjects() const {
    return objects;
  }

  Object_recognition(const std::string object_folder_path);
  void getObjects(const PointCloudT::Ptr& scene, std::vector<Result>& object_poses);
  void showPointCloud(PointCloudT::Ptr cloud, char* name);
private:
  bool getObjectPointsOnPlane(const PointCloudT::Ptr &scene, PointCloudT::Ptr &objects_pose);
  void setObjects(const std::string object_folder_path);
  void segmentSceneInObjects(const PointCloudT::Ptr& cloud, std::vector<Feature_cloud> &scene_objects);
  void alignment(const std::vector<Feature_cloud>& scene_objects, std::vector<Result>& objects_pose);
  void refineAlignment(PointCloudT::Ptr query, PointCloudT::Ptr target, Result res);
  std::vector<Feature_cloud> objects;
  double max_z;
};

#endif // OBJECT_RECOGNITION_H
