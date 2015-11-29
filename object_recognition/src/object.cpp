#include "object.h"
#include <pcl/io/obj_io.h>
#include <fstream>
using std::ifstream;
Object::Object(const std::string& object_folder_path, const std::string& file_name)
  : Feature_cloud(file_name.substr(0,file_name.find_last_of(".")))
{
  CloudT::Ptr xyz(new CloudT);
  pcl::io::loadOBJFile<PointT>(object_folder_path+"/" + file_name, *xyz);
  for (pcl::PointCloud<PointT>::iterator i = xyz->begin(); i != xyz->end(); i++)
  {
      i->z *= mm_to_m_factor; i->y *= mm_to_m_factor; i->x *= mm_to_m_factor;
  }
  computeFeatures(xyz);
  setGrasps(object_folder_path);
}

void Object::setGrasps(const string& object_folder_path)
{
  const string grasp_folder_name = "grasps";
  string grasp_file_path = object_folder_path + '/' + grasp_folder_name + '/' + name + ".txt";
  cout << "loading grasps from: " << grasp_file_path << endl;
  std::ifstream fs(grasp_file_path.c_str());
  if(fs.is_open())
  {
    while(fs.good())
    {
      Eigen::Matrix4f grasp;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          fs >> grasp(i,j);
        }
      }
      cout << endl << name << "'s grasp = " << endl << grasp << endl;
      grasps.push_back(grasp);
    }
    grasps.pop_back(); // one too much added??
    fs.close();
  }
  else {
    PCL_ERROR("Unable to open grasp file\n");
  }
}
