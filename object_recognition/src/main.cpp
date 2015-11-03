#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/image_grabber.h>
#include <pcl/point_types.h>
// for extract object plane
#include <object_recognition.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/point_cloud_image_extractors.h>

using std::cout;
using std::endl;
using namespace cv;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
void readScene (char* file_name, pcl::PointCloud<PointT>::Ptr scene);

int main()
{
  pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  char* im_file_name = "../resources/scene/vis3_test_image.png";
  char *obj_folder_path("../resources/objects");
  Mat im = cv::imread(im_file_name, cv::IMREAD_COLOR);

  cout << "Starting object detection" << endl;
  char* file_name = "../resources/scene/table_objects.pcd";
  pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
  readScene(file_name, scene);
  std::vector<CloudT::Ptr> scene_objects;
  Object_recognition obj_recogizer(obj_folder_path);
  //obj_recogizer.displaySegments(scene);
  vector<Object_recognition::Result> object_poses;
  PCL_INFO("Determine object poses\n");
  obj_recogizer.getObjects(scene, object_poses);
  obj_recogizer.displayAlignment(scene, object_poses);
  PCL_DEBUG("Done Object detection\n");
  return 0;
}


void readScene (char* file_name, pcl::PointCloud<PointT>::Ptr scene)
{
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile (file_name, cloud_blob) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", file_name);
    return;
  }
  pcl::fromPCLPointCloud2 (cloud_blob, *scene); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
}
