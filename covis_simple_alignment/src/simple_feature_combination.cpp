#define PCL_NO_PRECOMPILE
#include <pcl/point_traits.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <algorithm>

#include<map>


#include <covis/covis.h>
#include "computeFeatures.h"
using namespace covis;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

using namespace Eigen;

typedef Eigen::Matrix4f MyMatrix;


// Point and feature type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBNormal PointN;

core::Correspondence::Vec best;


using namespace covis::core;
using namespace covis::feature;

const double resolution = 0.01f;

void downsamplePointCloud(pcl::PointCloud<PointN>::Ptr &cloud){
    // Create the filtering object
    pcl::VoxelGrid<PointN> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (resolution, resolution, resolution);
    sor.filter (*cloud);
}

void computeAndCorrectNormals(pcl::PointCloud<PointN>::Ptr &cloud){
    //    const size_t knn = 20;
    const float nrad = 0.01;
    std::vector<int> dummy;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, dummy);
    {
        // Compute normals
        ScopedTimer t("Normals");
        pcl::NormalEstimationOMP<PointN, PointN> ne;
        ne.setInputCloud(cloud);
        ne.setRadiusSearch(nrad);
        ne.compute(*cloud);
    }
}


int main(int argc, const char** argv) {
    //core::ProgramOptions po;
    //po.addPositional("pcd-query", "point cloud file for query point cloud");
    //po.addPositional("pcd-target", "point cloud file for target point cloud");

    //if(!po.parse(argc, argv))
    //    return 1;
    {
        covis::core::ScopedTimer t("Begins - ends");
        //-------------------------------------------------------------------------------------------------------------------------------
        pcl::PointCloud<PointN>::Ptr target_org(new pcl::PointCloud<PointN>());
        //pcl::io::loadPCDFile<PointN>(po.getValue("pcd-target"), *target_org);
        pcl::io::loadPCDFile<PointN>("../resources/object.pcd", *target_org);
        double scaleFactor = 0.001;
        for (pcl::PointCloud<PointN>::iterator i = target_org->begin(); i != target_org->end(); i++)
        {
            i->z *= scaleFactor; i->y *= scaleFactor; i->x *= scaleFactor;
        }
        pcl::PointCloud<PointN>::Ptr target(new pcl::PointCloud<PointN>());
        pcl::copyPointCloud(*target_org, *target);

        //-------------------
        computeAndCorrectNormals(target);
        downsamplePointCloud(target);
        //        target = thresholdEdgePoints(target);

        pcl::PointCloud<PointN>::Ptr query_org(new pcl::PointCloud<PointN>());
        //pcl::io::loadPCDFile<PointN>(po.getValue("pcd-query"), *query_org);
        pcl::io::loadPCDFile<PointN>("../resources/scene.pcd", *query_org);
        pcl::PointCloud<PointN>::Ptr query(new pcl::PointCloud<PointN>());
        pcl::copyPointCloud(*query_org, *query);
        computeAndCorrectNormals(query);
        downsamplePointCloud(query);
        //        query = thresholdEdgePoints(query);

        ComputeFeatures cf;
        cf.setQuery(query);
        cf.setTarget(target);
        cf.setVizualizeCorrespondences(true);
        cf.setVizualizeDetection(true);
        //    core::Correspondence::Vec corrSHOT = cf.computeShot(0.03);
        core::Correspondence::Vec corrSHOT = cf.computeCShot(0.05);
        core::Correspondence::Vec corrECSAD = cf.computeECSAD(0.05);

        core::Correspondence::Vec corr_concatinated;
        for (size_t j = 0; j < corrSHOT.size(); j++){
            corr_concatinated.push_back(corrSHOT[j]);
        }
        for (size_t j = 0; j < corrECSAD.size(); j++){
            corr_concatinated.push_back(corrECSAD[j]);
        }
        cf.computePoseAndersRansac(corr_concatinated, query, target);
        //POSSIBLE THINGS TO DO
//        std::vector<int> dummy;
//        pcl::removeNaNFromPointCloud(*xyz, *xyz, dummy);
//
//        pcl::PointCloud<PointTN>::Ptr xyz_normal(new  pcl::PointCloud<PointTN>());
//        pcl::PointCloud<PointTN>::Ptr xyz_normal_voxeled(new  pcl::PointCloud<PointTN>());
//        computeNormals(xyz, xyz_normal);
//        voxelGrid(xyz_normal, xyz_normal_voxeled);
//        //    pcl::io::savePCDFile("ouch.pcd", *xyz_normal_voxeled);
//
//        features_ = getSHOT(xyz_normal_voxeled, 0.01);
//        //make nan to 0
//        for (size_t i = 0; i < features_->size(); i++){
//            for (size_t j = 0; j < 352; j++){
//                if (!pcl_isfinite(features_->points[i].descriptor[j])) {
//                    features_->points[i].descriptor[j] = 0;
//                }
//            }
//        }

    }
    return 0;
}
