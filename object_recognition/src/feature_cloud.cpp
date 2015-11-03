#include "feature_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>

Feature_cloud::Feature_cloud(CloudT::Ptr xyz, std::string name="Unknown obj", bool inMM=false)
    : resolution_(0.001f),
      search_method_(new SearchMethodT),xyz_(new CloudT), features_(new FeatureCloudT),
      name(name)
{    
    CloudT::Ptr tmp(new CloudT);
    pcl::copyPointCloud(*xyz, *tmp);
    if(inMM) {

        int scaleFactor = 0.01;
        for (pcl::PointCloud<PointT>::iterator i = tmp->begin(); i != tmp->end(); i++)
        {
            i->z *= scaleFactor; i->y *= scaleFactor; i->x *= scaleFactor;
        }

    }
    normal_radius_ = 2*resolution_;
    feature_radius_ = 25*resolution_;
    computeFeatures(tmp);
}

void Feature_cloud::computeFeatures(CloudT::Ptr xyz)
{
    PCL_WARN("Point cloud contains %d points\n", xyz->points.size());
    /* Down sampling to fixed resolution
     * http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid
     */
    PCL_WARN("Downsampling\n");
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


    PCL_WARN("Point cloud contains %d points\n", xyz_->points.size());
    if(xyz_->points.size() < 100) {
        PCL_ERROR("Less then ten points in cloud, cloud size=%i",xyz_->points.size());
        {
            const float search_radius = 0.1f; // 10cm
            // Filtering object.
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
            filter.setInputCloud(xyz);
            // Object for searching.
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
            filter.setSearchMethod(kdtree);
            // Use all neighbors in a radius of 3cm.
            filter.setSearchRadius(search_radius);
            // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
            // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
            filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
            // Radius around each point, where the local plane will be sampled.
            filter.setUpsamplingRadius(search_radius);
            // Sampling step size. Bigger values will yield less (if any) new points.
            filter.setUpsamplingStepSize(resolution_);
            filter.process(*xyz_);
        }
        //throw 1;
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
        ne.setSearchSurface(xyz_);
        ne.compute(*surfNormal);
    }
    PCL_WARN( "%d normals computed\n", surfNormal->points.size() );
    PCL_WARN("Point cloud contains %d points\n", xyz_->points.size());
    // Check normals for NaN or infinity
    PCL_WARN("Checking normals...\n");
    //surfNormal->back().
    pcl::PointCloud<pcl::PointXYZ>::iterator j = xyz_->begin();
    for (pcl::PointCloud<pcl::Normal>::iterator i = surfNormal->begin(); i != surfNormal->end()-1; )
    {
        if ( pcl_isfinite(i->normal_x) && pcl_isfinite(i->normal_y) && pcl_isfinite(i->normal_z))
        {
            i++;
            j++;
        }
        else
        {
            surfNormal->erase( i );
            xyz_->erase( j );
            i++;
            j++;
            PCL_ERROR("Normal is infinite!!!\n");
        }
    }
    if(xyz_->points.size() < 10) {
        PCL_ERROR("Less then ten finite normals in cloud, cloud size=%i",surfNormal->points.size());
        throw 2;
    }
    // For debugging, output the size of both clouds
    PCL_WARN("Point cloud contains %d points\n", xyz_->points.size());
    PCL_WARN("Normals contains %d points\n", surfNormal->points.size());
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
    PCL_WARN("Checking feature for infinite values...\n");
    j = xyz_->begin();
    for (pcl::PointCloud<FeatureT>::iterator i = features_->begin(); i != features_->end()-1; )
    {
        bool validDescriptor=true;
        for (int k = 0; k < i->descriptorSize(); ++k) {
            if(!pcl_isfinite(i->descriptor[k])) {
                validDescriptor = false;
                break;
            }
        }
        if (validDescriptor)
        {
            i++;
            j++;
        }
        else
        {
            features_->erase( i );
            xyz_->erase( j );
            i++;
            j++;
            PCL_ERROR("Feature is infinite!!!\n");
        }
    }
    if(xyz_->points.size() < 10) {
        PCL_ERROR("Less then ten finite normals in cloud, cloud size=%i",features_->points.size());
        throw 2;
    }
}
