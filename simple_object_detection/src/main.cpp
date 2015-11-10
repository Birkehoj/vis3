#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <string>
#include <pcl/registration/ia_ransac.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormalT;
typedef pcl::SHOT352 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
using namespace std;
void readCloud (string file_name, CloudT::Ptr scene);
void computeNormals(SurfaceNormalT::Ptr surfNormal, CloudT::Ptr xyz_);
void computeFeatures(CloudT::Ptr xyz_, FeatureCloudT::Ptr features_);
void downsamplePointCloud(CloudT::Ptr &cloud);
void displayAlignment(const CloudT::Ptr &scene, const CloudT::Ptr &aligned_object);
void showPointCloud(const CloudT::Ptr& cloud, char* name);
double resolution_(0.01f);
int main()
{
  CloudT::Ptr scene_cloud(new CloudT);
  readCloud("../resources/scene.pcd",scene_cloud);
  showPointCloud(scene_cloud,"scene cloud");
  CloudT::Ptr object_cloud(new CloudT);
  readCloud("../resources/object.pcd",object_cloud);
  double scaleFactor = 0.001;
  for (pcl::PointCloud<PointT>::iterator i = object_cloud->begin(); i != object_cloud->end(); i++)
  {
      i->z *= scaleFactor; i->y *= scaleFactor; i->x *= scaleFactor;
  }
  showPointCloud(object_cloud,"object cloud");
  downsamplePointCloud(scene_cloud);
  cout << "scene size " << scene_cloud->size() << endl;
  downsamplePointCloud(object_cloud);
  cout << "object size " << object_cloud->size() << endl;
  FeatureCloudT::Ptr scene_cloud_feature(new FeatureCloudT);
  computeFeatures(scene_cloud, scene_cloud_feature);
  FeatureCloudT::Ptr object_feature(new FeatureCloudT);
  computeFeatures(object_cloud, object_feature);

  double min_sample_distance_ (0.01f);
  double max_correspondence_distance_(1);//(0.1f*0.1f);
  double nr_iterations_ (1000);
  pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::SHOT352> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance_);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance_);
  sac_ia.setMaximumIterations (nr_iterations_);
  sac_ia.setRANSACIterations(50000);
  //sac_ia.setCorrespondenceEstimation();
  sac_ia.setInputTarget(object_cloud);
  sac_ia.setTargetFeatures(object_feature);

  sac_ia.setInputCloud(scene_cloud);
  sac_ia.setSourceFeatures(scene_cloud_feature);
  CloudT::Ptr registration_output(new CloudT);
  sac_ia.align (*registration_output);
  cout << "transformed size " << registration_output->size() << endl;
  displayAlignment(scene_cloud, registration_output);
  return 0;
}

void readCloud (string file_name, CloudT::Ptr scene)
{
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile (file_name, cloud_blob) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", file_name.c_str());
    return;
  }
  pcl::fromPCLPointCloud2 (cloud_blob, *scene); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
}

/* Compute surface normals
 * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
 */
void computeNormals(SurfaceNormalT::Ptr surfNormal, CloudT::Ptr xyz_)
{
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    //ne.setRadiusSearch(normal_radius_);
    ne.setNumberOfThreads(3);
    ne.setKSearch(10);
    ne.setInputCloud(xyz_);
    ne.setSearchSurface(xyz_);
    ne.compute(*surfNormal);
}

void computeFeatures(CloudT::Ptr xyz_, FeatureCloudT::Ptr features_)
{

  double feature_radius_ = 0.05;//25.0f*resolution_; //
  SurfaceNormalT::Ptr surfNormal(new SurfaceNormalT);
  computeNormals(surfNormal, xyz_);
  pcl::SHOTEstimationOMP<PointT,pcl::Normal,FeatureT> shot;
  shot.setNumberOfThreads(3);
  shot.setRadiusSearch(feature_radius_);
  shot.setSearchSurface(xyz_);
  // shot.setSearchMethod(search_method_); // No match ??
  shot.setInputNormals(surfNormal);
  shot.setInputCloud(xyz_);
  shot.compute(*features_);

  PCL_WARN("Checking feature for infinite values...\n");
  for (size_t i = 0; i < features_->size(); i++){
      for (size_t j = 0; j < features_->points[i].descriptorSize(); j++){
          if (!pcl_isfinite(features_->points[i].descriptor[j])) {
              features_->points[i].descriptor[j] = 0;
              PCL_WARN("Feature is infinite!!!\n");
          }
      }
  }
}

void downsamplePointCloud(CloudT::Ptr &cloud){
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (resolution_, resolution_, resolution_);
    sor.filter (*cloud);
}

void displayAlignment(const CloudT::Ptr &scene, const CloudT::Ptr &aligned_object)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("alignment"));
  viewer->setBackgroundColor (0, 0, 0);
  PCL_INFO("Displaying alignment for object in the scene:\n");
  viewer->addPointCloud (scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene" );
  viewer->addPointCloud (aligned_object, ColorHandlerT(aligned_object, 255.0, 0.0, 0.0), "object" );
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
  // Show alignmen
}

void showPointCloud(const CloudT::Ptr& cloud, char* name)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
}