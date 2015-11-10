/*
 * computeFeatures.cpp
 *
 *  Created on: Jul 27, 2015
 *      Author: lilita
 */

typedef pcl::SHOT352 FeatureSHOT;
typedef pcl::SHOT1344 FeatureCSHOT;
typedef pcl::ReferenceFrame RFT;
typedef covis::feature::ECSAD<PointN>::Histogram FeatureECSAD;

using namespace covis;
#include "computeFeatures.h"
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>

pcl::PointCloud<FeatureSHOT>::Ptr getSHOT(float radius, pcl::PointCloud<PointN>::Ptr cloud){

    pcl::SHOTEstimationOMP<PointN,PointN,FeatureSHOT,RFT> fest;
    fest.setRadiusSearch(radius);
    pcl::PointCloud<FeatureSHOT>::Ptr shotHist (new pcl::PointCloud<FeatureSHOT>);
    fest.setInputCloud(cloud);
    fest.setInputNormals(cloud);
    fest.compute(*shotHist);

    return shotHist;
}


covis::core::Correspondence::Vec ComputeFeatures::computeShot(float radius){
    pcl::PointCloud<FeatureSHOT>::Ptr fquery = getSHOT(radius, this->query);
    //make nan to 0
    for (size_t i = 0; i < fquery->size(); i++){
        for (size_t j = 0; j < 352; j++){
            if (!pcl_isfinite(fquery->points[i].descriptor[j])) {
                fquery->points[i].descriptor[j] = 0;
            }
        }
    }

    pcl::PointCloud<FeatureSHOT>::Ptr ftarget = getSHOT(radius, this->target);
    //make nan to 0
    for (size_t i = 0; i < ftarget->size(); i++){
        for (size_t j = 0; j < 352; j++){
            if (!pcl_isfinite(ftarget->points[i].descriptor[j])) {
                ftarget->points[i].descriptor[j] = 0;
            }
        }
    }

    //compute corresponding points
    covis::core::Correspondence::Vec corr, best;
    {
        covis::core::ScopedTimer t("core::Correspondence::Vec SHOT features");
        pcl::console::print_warn("Amount of fquery points: %d and ftarget: %d\n", fquery->size(), ftarget->size());
        corr = *detect::computeKnnMatches<FeatureSHOT>(fquery, ftarget, 1);
        pcl::console::print_warn("Amount of corr: %d\n", corr.size());

    }
    if (this->vizualizeCorrespondences) {
        best = corr;
        core::sort(best);
        best.resize(100);
        covis::visu::showCorrespondences<PointN>(this->query, this->target, best);
    }
    return corr;
}
///-------------------------------------
pcl::PointCloud<FeatureCSHOT>::Ptr getCSHOT(float radius, pcl::PointCloud<PointN>::Ptr cloud){

    pcl::SHOTColorEstimationOMP<PointN,PointN,FeatureCSHOT,RFT> fest;
    fest.setRadiusSearch(radius);
    pcl::PointCloud<FeatureCSHOT>::Ptr shotHist (new pcl::PointCloud<FeatureCSHOT>);
    fest.setInputCloud(cloud);
    fest.setInputNormals(cloud);
    fest.compute(*shotHist);

    return shotHist;
}


covis::core::Correspondence::Vec ComputeFeatures::computeCShot(float radius){
    covis::core::Correspondence::Vec corr, best;
    {
        covis::core::ScopedTimer t("core::Correspondence::Vec SHOT features");
        pcl::PointCloud<FeatureCSHOT>::Ptr fquery = getCSHOT(radius, this->query);
        //make nan to 0
        for (size_t i = 0; i < fquery->size(); i++){
            for (size_t j = 0; j < 352; j++){
                if (!pcl_isfinite(fquery->points[i].descriptor[j])) {
                    fquery->points[i].descriptor[j] = 0;
                }
            }
        }

        pcl::PointCloud<FeatureCSHOT>::Ptr ftarget = getCSHOT(radius, this->target);
        //make nan to 0
        for (size_t i = 0; i < ftarget->size(); i++){
            for (size_t j = 0; j < 352; j++){
                if (!pcl_isfinite(ftarget->points[i].descriptor[j])) {
                    ftarget->points[i].descriptor[j] = 0;
                }
            }
        }

        //compute corresponding points


        pcl::console::print_warn("Amount of fquery points: %d and ftarget: %d\n", fquery->size(), ftarget->size());
        corr = *detect::computeKnnMatches<FeatureCSHOT>(fquery, ftarget, 1);
        pcl::console::print_warn("Amount of corr: %d\n", corr.size());


        if (this->vizualizeCorrespondences) {
            best = corr;
            core::sort(best);
            best.resize(100);
            covis::visu::showCorrespondences<PointN>(this->query, this->target, best);
        }
    }
    return corr;
}
//-----------------------------------------------------------------------------------------------------------
pcl::PointCloud<FeatureECSAD>::Ptr getECSAD(pcl::PointCloud<PointN>::Ptr cloud, float radius){
    feature::ECSAD<PointN> ecsad;
    ecsad.setRadius(radius);
    //    ecsad.setSurface(*cloud);
    pcl::PointCloud<FeatureECSAD>::Ptr ecsadHist = ecsad.compute(cloud);
    return ecsadHist;
}

covis::core::Correspondence::Vec ComputeFeatures::computeECSAD(float radius){
    core::Correspondence::Vec corr;
    {
        covis::core::ScopedTimer t("ECSAD features");
        pcl::PointCloud<FeatureECSAD>::Ptr fquery = getECSAD(this->query,  radius);
        pcl::PointCloud<FeatureECSAD>::Ptr ftarget = getECSAD(this->target,  radius);

        corr = *detect::computeKnnMatches<FeatureECSAD>(fquery, ftarget, 1);
        core::Correspondence::Vec best;
        if (this->vizualizeCorrespondences) {
            best = corr;
            core::sort(best);
            best.resize(100);
            visu::showCorrespondences<PointN>(this->query, this->target, best);
        }
    }
    return corr;
}
//--------------------------------------------------------------------------------------------------------------
//Pose estimation tools
void computeCube(Eigen::Vector3f& tfinal,  Eigen::Quaternionf& qfinal_r, pcl::PointCloud<PointN>::Ptr final, float& _x, float& _y, float& _z) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*final, centroid);
    std::cout << "centroid: \n" << centroid[0] << " " << centroid[1] << " " << centroid[2]<< std::endl;

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*final, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointN> cPoints;
    pcl::transformPointCloud(*final, cPoints, p2w);
    PointN min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    // final transform

    const Eigen::Quaternionf qfinal(eigDx);

    tfinal = eigDx*mean_diag + centroid.head<3>();
    std::cout << "tfinal: " << tfinal << std::endl;
    pcl::PointXYZRGB minp, maxp;
    Eigen::Matrix4f _tr = Eigen::Matrix4f::Identity();
    _tr.topLeftCorner<3,3>() = qfinal.toRotationMatrix();
    _tr.block<3,1>(0,3) = tfinal;


    _x = (max_pt.x-min_pt.x);// * 0.5;
    _y = (max_pt.y-min_pt.y);// * 0.5;
    _z = (max_pt.z-min_pt.z);// * 0.5;

    _tr = _tr.inverse().eval();
    qfinal_r = qfinal;


    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
    for (size_t i = 0; i < final->size(); i++){
        PointN p = (*final)[i];

        p.getVector4fMap() = _tr * p.getVector4fMap();

        if(fabsf(p.x) <= _x &&
                fabsf(p.y) <= _y &&
                fabsf(p.z) <= _z ) {
            object_indices->indices.push_back(i);
        }
    }
    pcl::PointCloud<PointN>::Ptr small_cube(new pcl::PointCloud<PointN>());

    small_cube->height = 1;
    small_cube->width = object_indices->indices.size();
    small_cube->points.resize(small_cube->height * small_cube->width);

    pcl::copyPointCloud (*final , object_indices->indices, *small_cube );
    pcl::io::savePCDFileBinary("/home/lilita/covis/covis-app/getStereo/build/object_tracking_template.pcd", *small_cube);
    pcl::console::print_error("object_tracking_template saved \n");
}

void display(const pcl::PointCloud<PointN>::Ptr target, const pcl::PointCloud<PointN>::Ptr final){

    //add cube around the object
    Eigen::Vector3f tfinal;
    Eigen::Quaternionf qfinal;
    float _x = 0, _y = 0, _z = 0;
    computeCube(tfinal, qfinal, final, _x, _y, _z);

    pcl::visualization::PCLVisualizer viewer ("Alignment");

    pcl::visualization::PointCloudColorHandlerRGBField<PointN> rgb(target);

    viewer.addPointCloud<PointN> (target, rgb, "shortEdgeT");

    //    viewer.addPointCloud<PointN> (final, "final");
    //    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "final");
    viewer.addCube(tfinal, qfinal, _x, _y, _z, "cube", 0);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}

void ComputeFeatures::computePoseAndersRansac(core::Correspondence::Vec corr, const pcl::PointCloud<PointN>::Ptr query,
        const pcl::PointCloud<PointN>::Ptr target ){

    core::Correspondence::VecPtr v(new core::Correspondence::Vec(corr));

    detect::Ransac<PointN> r;
    r.setSource(query);
    r.setTarget(target);
    r.setCorrespondences(v);
    r.setIterations(50000);
    r.setInlierThreshold(0.1);
    r.setFullEvaluation(false);
    r.setInlierFraction(0);
    Eigen::Matrix4f m = r.estimate().pose;

    pcl::PointCloud<PointN>::Ptr after_ransac(new pcl::PointCloud<PointN>());
    //ICP
    pcl::transformPointCloud(*query, *after_ransac, m);


    pcl::PointCloud<PointN>::Ptr final(new pcl::PointCloud<PointN>());
/*
    pcl::IterativeClosestPoint<PointN, PointN> icp;
    icp.setInputSource(after_ransac);
    icp.setInputTarget(target);
    icp.setMaximumIterations(1000);
    icp.align(*final);
*/
    display(target, final);
}


