#include "feature_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/time.h>
Feature_cloud::Feature_cloud(CloudT::Ptr xyz, std::string name="Unknown obj", bool inMM=false)
    : resolution_(0.01f),
      search_method_(new SearchMethodT),xyz_(new CloudT), features_(new FeatureCloudT),
      name(name)
{    
    CloudT::Ptr tmp(new CloudT);
    pcl::copyPointCloud(*xyz, *tmp);
    if(inMM) {

        double scaleFactor = 0.001;
        for (pcl::PointCloud<PointT>::iterator i = tmp->begin(); i != tmp->end(); i++)
        {
            i->z *= scaleFactor; i->y *= scaleFactor; i->x *= scaleFactor;
        }
    }
    feature_radius_ = 5.0f*resolution_;
    //pcl::ScopeTime st("Feature extraction time");
    computeFeatures(tmp);
    //PCL_INFO("time to calc./feature: %f", st.getTimeSeconds()/features_->size());
}

void Feature_cloud::computeFeatures(CloudT::Ptr xyz)
{  
    /* Down sampling to fixed resolution
     * http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid
     */
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(resolution_, resolution_, resolution_);
        vg.setInputCloud(xyz);
        vg.filter(*xyz_);
        /*
        pcl::PointCloud<int> sampled_indices;
        pcl::UniformSampling<PointT> uniform_sampling;
        uniform_sampling.setInputCloud (xyz);
        uniform_sampling.setRadiusSearch (resolution_ );
        uniform_sampling.compute (sampled_indices);
        pcl::copyPointCloud (*xyz, sampled_indices.points, *xyz_);
        */
    }
    /* Compute surface normals
     * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
     */
    SurfaceNormalT::Ptr surfNormal(new SurfaceNormalT);
    {
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        //ne.setRadiusSearch(normal_radius_);
        ne.setNumberOfThreads(3);
        ne.setKSearch(10);
        ne.setInputCloud(xyz_);
        ne.setSearchSurface(xyz_);
        ne.compute(*surfNormal);
    }
    /* Features extraction
     * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
     */
    pcl::SHOTEstimationOMP<PointT,pcl::Normal,FeatureT> shot;
    shot.setNumberOfThreads(3);
    shot.setRadiusSearch(feature_radius_);
    shot.setSearchSurface(xyz_);
    // shot.setSearchMethod(search_method_); // No match ??
    shot.setInputNormals(surfNormal);
    shot.setInputCloud(xyz_);    
    shot.compute(*features_);
    for (size_t i = 0; i < features_->size(); i++){
        for (size_t j = 0; j < features_->points[i].descriptorSize(); j++){
            if (!pcl_isfinite(features_->points[i].descriptor[j])) {
                features_->points[i].descriptor[j] = 0;
                PCL_WARN("Feature is infinite!!!\n");
            }
        }
    }
}
