#ifndef OBJECT_H
#define OBJECT_H
#include <feature_cloud.h>
#include <object_recognition_types.h>
class Object : public Feature_cloud
{
public:
  Object(const std::string& object_folder_path, const std::string& file_name);
  const vector<Transformation>& getGrasps() const {
    return grasps;
  }
private:
  const static double mm_to_m_factor = 0.001;
  vector<Transformation> grasps;
  void setGrasps(const string& object_folder_path);
};

#endif // OBJECT_H
