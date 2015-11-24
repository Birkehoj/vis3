#ifndef CHOOSE_OBJECT
#define CHOOSE_OBJECT
#include <object_recognition.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/common/time.h>
void showObjectBB(Mat im, pcl::PointCloud<PointT>::Ptr scene);
void choose_object(Mat scene_im, const vector<Object_recognition::Result>& objects, int& choosen_object_i)
{
  CloudT::Ptr object_cloud = objects.at(0).transformed_cloud;
  showObjectBB(scene_im, object_cloud);

}

void showObjectBB(Mat im, pcl::PointCloud<PointT>::Ptr scene)
{
  double cx = 319.5, cy = 239.5, f = 525.0;
  for (size_t i = 0; i < scene->size(); ++i) {
    int x = scene->points[i].x/scene->points[i].z * f + cx;
    int y = scene->points[i].y/scene->points[i].z * f + cy;
    im.at<Vec3b>(y,x) = Vec3b(0,0,255);
    cout << "(" << x << ", " << y << ")" << endl;
  }
  imshow("bb",im);
  waitKey();
}

#endif // CHOOSE_OBJECT

