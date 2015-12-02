#include "feature_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/time.h>
Feature_cloud::Feature_cloud(std::string name)
  : resolution_(0.01f),
    search_method_(new SearchMethodT),xyz_(new CloudT), features_(new FeatureCloudT),
    name(name),  feature_radius_(10.0*resolution_)
{}

Feature_cloud::Feature_cloud(CloudT::Ptr xyz, std::string name="Unknown obj")
  : Feature_cloud(name)
{
  computeFeatures(xyz);
}

void Feature_cloud::computeFeatures(CloudT::Ptr xyz)
{  
  //pcl::ScopeTime st("Feature extraction time");
  /* Down sampling to fixed resolution
     * http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid
     */
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(resolution_, resolution_, resolution_);
    vg.setInputCloud(xyz);
    vg.filter(*xyz_);
  }
  /* Compute surface normals
     * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
     */
  SurfaceNormalT::Ptr surfNormal(new SurfaceNormalT);
  {
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    //ne.setRadiusSearch(normal_radius_);
    ne.setNumberOfThreads(4);
    //ne.setKSearch(10);
    ne.setRadiusSearch(feature_radius_);
    ne.setInputCloud(xyz_);
    ne.setSearchSurface(xyz_);
    ne.compute(*surfNormal);
  } 
  /*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(xyz_, surfNormal);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
  */
  /* Features extraction
   * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
   */

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud (xyz_);
  fpfh_est.setInputNormals (surfNormal);
  //fpfh_est.setSearchMethod (search_method_);
  fpfh_est.setRadiusSearch (feature_radius_);
  fpfh_est.compute (*features_);

  /*
  pcl::SHOTEstimationOMP<PointT,pcl::Normal,FeatureT> shot;
  shot.setNumberOfThreads(4);
  shot.setRadiusSearch(feature_radius_);
  shot.setSearchSurface(xyz_);
  // shot.setSearchMethod(search_method_); // No match ??
  shot.setInputNormals(surfNormal);
  shot.setInputCloud(xyz_);
  shot.compute(*features_);
  */
  int invalid_feature_count = 0; // debug
  for (size_t i = 0; i < features_->size(); i++){
    for (size_t j = 0; j < features_->points[i].descriptorSize(); j++){
      /*
      if (!pcl_isfinite(features_->points[i].descriptor[j])) {
        features_->points[i].descriptor[j] = 0;
      */
      if (!pcl_isfinite(features_->points[i].histogram[j])) {
              features_->points[i].histogram[j] = 0;
        invalid_feature_count++;
      }
    }
  }
  PCL_WARN("Oject; %s, has %i infinite features\n", name.c_str(), invalid_feature_count);
  //PCL_INFO("time to calc./feature: %f", st.getTimeSeconds()/features_->size());
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Feature_cloud::normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
  viewer->addPointCloud (cloud, ColorHandlerT(cloud, 255.0, 0.0, 0.0), "sample cloud" );
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  return (viewer);
}
