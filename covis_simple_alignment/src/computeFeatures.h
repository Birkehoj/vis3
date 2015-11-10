/*
 * computeFeatures.h
 *
 *  Created on: Jul 27, 2015
 *      Author: lilita
 */

#ifndef COMPUTEFEATURES_H_
#define COMPUTEFEATURES_H_

typedef pcl::PointXYZRGBNormal PointN;
#include <covis/covis.h>

class ComputeFeatures {
    private:
        pcl::PointCloud<PointN>::Ptr query;
        pcl::PointCloud<PointN>::Ptr target;
        bool vizualizeCorrespondences;
        bool vizualizeDetection;

    public:
        ComputeFeatures() {};
        virtual ~ComputeFeatures() {};

        covis::core::Correspondence::Vec computeShot(float radius);
        covis::core::Correspondence::Vec computeCShot(float radius);

        covis::core::Correspondence::Vec computeECSAD(float radius);

        void computePoseAndersRansac(covis::core::Correspondence::Vec corr, const pcl::PointCloud<PointN>::Ptr query,
                const pcl::PointCloud<PointN>::Ptr target );

        void setQuery(pcl::PointCloud<PointN>::Ptr cloud){
            this->query = cloud;
        }
        void setTarget(pcl::PointCloud<PointN>::Ptr cloud){
            this->target = cloud;
        }
        void setVizualizeCorrespondences(bool val){
            this->vizualizeCorrespondences = val;
        }
        void setVizualizeDetection(bool val){
            this->vizualizeDetection = val;
        }
};

#include "computeFeatures.hpp"
#endif /* COMPUTEFEATURES_H_ */
