#include "object_recognition.h"
#include <dirent.h>
#include <pcl/io/obj_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <stdlib.h> // for itoa
#include <pcl/common/time.h>
#include <algorithm>
Object_recognition::Object_recognition(const std::string object_folder_path)
  : object_folder_path(object_folder_path)
{    
  setObjects();
  setObjectsSaC();
}

void Object_recognition::setObjects()
{
  PCL_INFO("Loading objects from [%s]\n",object_folder_path.c_str());
  struct dirent *pDirent;
  DIR *pDir;
  pDir = opendir ((object_folder_path).c_str());
  if (pDir == NULL) {
    printf ("Cannot open directory '%s'\n", object_folder_path.c_str());
    return;
  }
  char *obj_file_end = "obj"; // "mtl"
  while ((pDirent = readdir(pDir)) != NULL) {
    if(!std::strcmp( pDirent->d_name, "." ) || !std::strcmp( pDirent->d_name, "..")
       || !std::strcmp( pDirent->d_name,"./coke_can.obj.obj") )
      continue;
    std::string fn(pDirent->d_name);
    if(fn.substr(fn.find_last_of(".") + 1) == obj_file_end) { // && !std::strcmp( pDirent->d_name,"BakingVanilla_25k_tex.obj")
      printf ("Loading [%s]\n", pDirent->d_name);
      objects.push_back(Object(object_folder_path, fn));
      PCL_INFO("added point cloud: %s  ", objects.back().name.c_str());
      PCL_INFO("with %i features\n", objects.back().getLocalFeatures()->size());
      //pcl::io::savePCDFile((objects.back().name + ".pcd").c_str(), *objects.back().getCloud());
    }
  }
  closedir (pDir);
}

void Object_recognition::getObjects(const CloudT::Ptr& scene, vector<vector<Result> >& object_poses)
{    
  vector<Feature_cloud> scene_objects;
  {
    PCL_DEBUG("Segment objects");
    {
      vector<CloudT::Ptr> scene_clouds;
      segmentSceneInObjects(scene, scene_clouds);
      getSceneSegmentFeatures(scene_clouds, scene_objects);
    }
  }
  PCL_DEBUG("Align objects");
  alignment(scene_objects, object_poses);
}

void Object_recognition::getSceneSegmentFeatures(const std::vector<CloudT::Ptr>& scene_clouds, std::vector<Feature_cloud>& scene_objects)
{
  for (int i = 0; i < scene_clouds.size(); ++i) {
    std::string cloud_name("scene_cloud"); cloud_name += (char(i) + '0');
    try{
      scene_objects.push_back(Feature_cloud(scene_clouds[i], cloud_name));
    } catch(...) {
      scene_objects.pop_back();
    }
    PCL_INFO("PointCloud representing the Cluster: %s, with %i data points\n", cloud_name.c_str(), scene_clouds[i]->points.size() );
  }
}

void Object_recognition::segmentSceneInObjects( const CloudT::Ptr& scene, std::vector<CloudT::Ptr>& scene_objects)
{  
  CloudT::Ptr cloud(new CloudT);
  pcl::copyPointCloud(*scene,*cloud);
  getObjectPointsOnPlane(scene, cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  scene_objects.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    scene_objects.push_back(cloud_cluster);
  }
}

void Object_recognition::showPointCloud(const CloudT::Ptr& cloud, char* name)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (name));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
}

void Object_recognition::displaySegments(const CloudT::Ptr &scene)
{
  std::vector<Feature_cloud> scene_objects;
  {
    vector<CloudT::Ptr> scene_clouds;
    segmentSceneInObjects(scene, scene_clouds);
    getSceneSegmentFeatures(scene_clouds, scene_objects);
    pcl::io::savePCDFile("scene.pcd", *scene_objects[0].getCloud());
  }
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("segments"));
  viewer->setBackgroundColor (0, 0, 0);
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
  PCL_DEBUG("Displaying segments for objects in the scene:\n");
  for (int i = 0; i < scene_objects.size(); ++i) {
    viewer->addPointCloud (scene_objects[i].getCloud(), ColorHandlerT(scene_objects[i].getCloud(), 0.0, 255.0, 0.0), scene_objects[i].name.c_str() );
    PCL_DEBUG("Object: %s\n", scene_objects[i].name.c_str());
    std::cout << std::endl;
    while (!viewer->wasStopped())
    {
      viewer->spinOnce();
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      // Do nothing but wait.
    }
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
  viewer->close();
}

void Object_recognition::displayAlignment(const CloudT::Ptr &scene, const vector<vector<Result> > &results)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("alignment"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointT> (scene, ColorHandlerT(scene, 50.0, 50.0, 50.0), "scene cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "scene cloud");
  PCL_INFO("Displaying alignment for objects in the scene:\n");
  //viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);  
  for (int i = 0; i < results.size(); ++i) {
    char buffer [10];
    sprintf(buffer, "%i", i);
    string str(buffer);
    string new_object_name = results[i][0].object_name + "_" + str;
    viewer->addPointCloud (results[i][0].transformed_cloud, ColorHandlerT(results[i][0].transformed_cloud, 0.0, 255.0, 0.0), new_object_name );
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, new_object_name);
    PCL_INFO("%s\n", new_object_name.c_str()); cout << endl;
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  viewer->close();
}

void Object_recognition::displayAlignment(const CloudT::Ptr &scene, const CloudT::Ptr &aligned_object)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("alignment"));
  viewer->setBackgroundColor (0, 0, 0); 
  viewer->addPointCloud (scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene" );
  viewer->addPointCloud (aligned_object, ColorHandlerT(aligned_object, 255.0, 0.0, 0.0), "object" );
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
  //viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters();
  viewer->setSize(640, 480);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // Do nothing but wait.
  }
  viewer->saveScreenshot("alignmet.png");
  viewer->close();
  // Show alignment
}

/* Initializes vector with initial alignment objects to avoid recal
 *
 */
void Object_recognition::setObjectsSaC()
{
  sac_ia_list.resize(objects.size());
  for(size_t i=0; i<objects.size(); i++)
  {
    pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT> sac_ia;
    sac_ia.setMinSampleDistance (0.025f);
    sac_ia.setCorrespondenceRandomness(10); // nearst neighbors to pick randomly between
    sac_ia.setMaxCorrespondenceDistance (0.01*0.01f);
    sac_ia.setMaximumIterations (3000);
    sac_ia.setInputSource(objects[i].getCloud());
    sac_ia.setSourceFeatures(objects[i].getLocalFeatures());
    //boost::shared_ptr<pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::HuberPenalty> error_functor(new pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::HuberPenalty(100.0f));
    //boost::shared_ptr<pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::TruncatedError> error_functor(new pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::TruncatedError(100.0f));
    // sac_ia.setErrorFunction(error_functor); // works different than default
    pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::KdTreeReciprocalPtr search_tree(new pcl::SampleConsensusInitialAlignment<PointT, PointT, Feature_cloud::FeatureT>::KdTreeReciprocal);
    sac_ia.setSearchMethodSource(search_tree, true);
    sac_ia_list[i] = sac_ia;
  }
}

void Object_recognition::alignment(const std::vector<Feature_cloud>& scene_objects, vector<vector<Result> >& objects_pose)
{
  pcl::ScopeTime st("Object recognition");
  objects_pose.resize(scene_objects.size());
  for(size_t j=0; j<scene_objects.size(); j++)
  {
    // Find the template with the best (lowest) fitness score
    PCL_INFO("%s :\n", scene_objects[j].name.c_str());
    Result object_match;
    object_match.score = std::numeric_limits<float>::infinity ();
    object_match.transformation = Transformation::Identity(4,4);
    vector<Result> results(objects.size());
    for(size_t i=0; i<objects.size(); i++)
    {
      sac_ia_list[i].setInputTarget(scene_objects[j].getCloud());
      sac_ia_list[i].setTargetFeatures(scene_objects[j].getLocalFeatures());
      pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output(new CloudT);
      sac_ia_list[i].align (*registration_output);
      double score = (double) sac_ia_list[i].getFitnessScore(sac_ia_list[i].getMaxCorrespondenceDistance());
      object_match.score = score;
      object_match.transformation = sac_ia_list[i].getFinalTransformation();
      object_match.transformed_cloud = registration_output;
      object_match.object_name = objects[i].name;
      refineAlignment(scene_objects[j], object_match, objects[i]);
      results[i] = object_match;
      /*
      if(objects[i].name == "YellowSaltCube_25k_tex" && j == 2)
        displayAlignment(scene_objects[j].getCloud(), object_match.transformed_cloud);
        */
    }
    std::sort(results.begin(), results.end(), Object_recognition::compareAlignmentResults); // sort after assending score
    objects_pose[j] = results;
  }
}

void
Object_recognition::refineAlignment(const Feature_cloud &query, Result &res, const Object &source)
{
  pcl::IterativeClosestPoint<PointT,PointT> icp;
  icp.setInputSource(source.getCloud());
  icp.setInputTarget(query.getCloud());
  icp.setMaximumIterations(50);
  icp.setMaxCorrespondenceDistance(0.005f);
  icp.setTransformationEpsilon (1e-8);
  icp.getSearchMethodTarget();
  pcl::PointCloud<PointT>::Ptr tmp(new CloudT);
  icp.align(*tmp, res.transformation);
  float max_range = 0.00005;
  if(icp.hasConverged()) {
    res.transformation = icp.getFinalTransformation();// * res.transformation;
    res.score = icp.getFitnessScore(max_range); // bad score
    pcl::copyPointCloud(*tmp, *res.transformed_cloud);
  } else {
    PCL_WARN("ICP failed!\n");
  }
  /*
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
               icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  */
  /*
  // Calculate matching score based on features from points that are aligned
  // NOT working propperly??
  if(icp.hasConverged())
  {
    double fitness_score = 0.0;

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    Feature_cloud::FeatureCloudT::Ptr target_features = query.getLocalFeatures();
    Feature_cloud::FeatureCloudT::Ptr source_features = source.getLocalFeatures();
    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < tmp->points.size (); ++i)
    {

      // Find its nearest neighbor in the target
      icp.getSearchMethodTarget()->nearestKSearch (tmp->points[i], 1, nn_indices, nn_dists);

      // Deal with occlusions (incomplete targets)
      if (nn_dists[0] > max_range*2)
        continue;
      // calculate feature difference
      Eigen::VectorXf p1(source_features->points[i].descriptorSize()), p2(source_features->points[i].descriptorSize());
      for (int k = 0; k < source_features->points[i].descriptorSize(); ++k) {
        if(source_features->points[i].histogram[k] < 1e8)
          p1[k] = source_features->points[i].histogram[k];
        if(target_features->points[i].histogram[k] < 1e8)
          p2[k] = target_features->points[nn_indices[0]].histogram[k];
      }
      // Calculate the fitness score
      fitness_score += fabs ((p1-p2).squaredNorm ());
      nr++;
    }

    if (nr > 0)
      fitness_score = (fitness_score / nr);
    else
      fitness_score = (std::numeric_limits<double>::max ());

    res.score = fitness_score;
  }
  */
}

void
Object_recognition::refineAlignment(const CloudT::Ptr &query, Result &res)
{
  pcl::IterativeClosestPoint<PointT,PointT> icp;
  icp.setInputSource(res.transformed_cloud);
  icp.setInputTarget(query);
  icp.setMaximumIterations(50);
  pcl::PointCloud<PointT>::Ptr tmp(new CloudT);
  icp.align(*tmp);
  if(icp.hasConverged()) {
    res.transformation = icp.getFinalTransformation() * res.transformation;
    res.score = icp.getFitnessScore(); // bad score
    pcl::copyPointCloud(*tmp, *res.transformed_cloud);
  } else {
    PCL_WARN("ICP failed!\n");
  }
  /*
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
               icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  */
}

bool
Object_recognition::getObjectPointsOnPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene, pcl::PointCloud<pcl::PointXYZ>::Ptr& objects)
{  
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (scene);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 2.0);
  pass.filter (*scene_filtered);  
  pass.setInputCloud (scene_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.6, 0.6);
  pass.filter (*scene_filtered);
  bool plane_found;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

  // Get the plane model, if present.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setInputCloud(scene_filtered);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.005);
  segmentation.setOptimizeCoefficients(true);
  segmentation.setMaxIterations(100);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  if (planeIndices->indices.size() == 0) {
    std::cout << "Could not find a plane in the scene." << std::endl;
    plane_found = false;
  }
  else
  {
    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(scene_filtered);
    extract.setIndices(planeIndices);
    extract.filter(*plane);
    // Retrieve the convex hull.
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(plane);
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(2);
    hull.reconstruct(*convexHull);
    // Redundant check.
    if (hull.getDimension() == 2)
    {
      // Prism object.
      pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
      prism.setInputCloud(scene_filtered);
      prism.setInputPlanarHull(convexHull);
      // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
      // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
      prism.setHeightLimits(0.01f, 0.2f);
      pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

      prism.segment(*objectIndices);
      // Get all points retrieved by the hull.
      extract.setIndices(objectIndices);
      extract.filter(*objects);
      //showPointCloud(objects, "points above plan");
      plane_found = true;
    }
    else {
      std::cout << "The chosen hull is not planar." << std::endl;
      plane_found = false;
    }
  }
  return plane_found;
}
