#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// for extract object plane
#include <object_recognition.h>
using std::cout;
using std::endl;
typedef pcl::PointXYZ PointT;
void readScene (char* file_name, pcl::PointCloud<PointT>::Ptr scene);

int main()
{

    cout << "Starting object detection" << endl;
    char* file_name = "../resources/scene/table_objects.pcd";
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    readScene(file_name, scene);
    /*
    */
    char *obj_folder_path("../resources/objects");
    Object_recognition obj_recogizer(obj_folder_path);
    vector<Object_recognition::Result> object_poses;
    obj_recogizer.getObjects(scene, object_poses);
    PCL_DEBUG("Done Object detection");
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
