#include "feature_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
Feature_cloud::Feature_cloud(CloudT::Ptr xyz)
    : normal_radius_ (0.06f), feature_radius_ (0.06f), resolution_(1.2f),
      search_method_(new SearchMethodT),xyz_(new CloudT), features_(new FeatureCloudT)

{
    computeFeatures(xyz);
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
    }

    /* Compute surface normals
     * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
     */
    SurfaceNormalT::Ptr surfNormal(new SurfaceNormalT);
    {
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        //ne.setRadiusSearch(normal_radius_);
        ne.setKSearch(10);
        ne.setInputCloud(xyz_);
        ne.setSearchSurface(xyz);
        ne.compute(*surfNormal);
    }

    /* Features extraction
     * http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
     */
    pcl::SHOTEstimationOMP<PointT,pcl::Normal,FeatureT> shot;
    shot.setRadiusSearch(feature_radius_);
    shot.setSearchSurface(xyz_);
    // shot.setSearchMethod(search_method_); // No match ??
    shot.setInputNormals(surfNormal);
    shot.setInputCloud(xyz_);
    shot.compute(*features_);
}
