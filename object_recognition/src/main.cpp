#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/image_grabber.h>
#include <pcl/point_types.h>
// for extract object plane
#include <object_recognition.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/common/time.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "object_recognition_types.h"
#include <stdio.h>
using namespace std;
using namespace cv;
void readScene (string file_name, pcl::PointCloud<PointT>::Ptr scene);
void pickGraspInImage(Mat scene_im, const vector<Object_recognition::Result>& results, const vector<Object>& objects);
void pickGraspInCloud(const vector<Object_recognition::Result>& results, const vector<Object>& objects, const string &scene_path_name);

const char ESC(27);
const char LEFT_ARROW(81);
const char UP_ARROW(82);
const char RIGHT_ARROW(83);
const char DOWN_ARROW(84);
const char CTRL(-28);
const char NUM_NULL(-80);
int main()
{
  //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
  //string im_file_name = "../resources/scene/five_objects_with_false/five_objects_with_false10.png";
  string im_file_name = "../resources/scene/complete_scene.png";
  string obj_folder_path("../resources/objects");

  Mat im = cv::imread(im_file_name, cv::IMREAD_COLOR);
  cout << "Starting object detection" << endl;
  //string file_name = "../resources/scene/five_objects_with_false/five_objects_with_false10.pcd";
  string file_name = "../resources/scene/complete_scene.pcd";
  pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
  readScene(file_name, scene);
  std::vector<CloudT::Ptr> scene_objects;
  Object_recognition obj_recogizer(obj_folder_path);
  //obj_recogizer.displaySegments(scene);
  vector<Object_recognition::Result> object_poses;
  PCL_INFO("Determine object poses\n");
  obj_recogizer.getObjects(scene, object_poses);
  obj_recogizer.displayAlignment(scene, object_poses);
  pickGraspInCloud(object_poses, obj_recogizer.getObjects(), file_name);
  pickGraspInImage(im, object_poses, obj_recogizer.getObjects());
  return 0;
}

inline Point3f pclPointToCv(const PointT& pcl_point)
{
  return Point3f(pcl_point.x, pcl_point.y, pcl_point.z);
}

void getAxisCloud(CloudT& axis_cloud)
{
  const double axis_length = 20e-3f;
  axis_cloud.push_back(PointT(0,0,0));
  axis_cloud.push_back(PointT(axis_length,0,0));
  axis_cloud.push_back(PointT(0,axis_length,0));
  axis_cloud.push_back(PointT(0,0,axis_length));
}

const Object& findObject(const vector<Object>& objects, const string& target_name)
{
  for (int i = 0; i < objects.size(); ++i)
  {
    if(objects[i].name == target_name)
    {
      return objects[i];
    }
  }
}

int result_size, grasp_size;
int result_i = 0, grasp_i = 0;
bool grasp_picked = false, aborted = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if(event.keyDown ())
  {
    string key_stroke = event.getKeySym();
    if(key_stroke == "Up")
    {
      result_i++;
      grasp_i = 0;
    } else if( key_stroke == "Down")
    {
      result_i--;
      grasp_i = 0;
    } else if( key_stroke == "Right")
    {
      grasp_i++;
    } else if( key_stroke == "Left")
    {
      grasp_i--;
    } else if( key_stroke == "Control_R")
    {
      grasp_picked = true;
    }
    else if( key_stroke == "KP_0" || key_stroke == "KP_Insert")
    {
      aborted = true;
    }
    // circular iterate through
    if(result_i < 0)
    {
      result_i = result_size - 1;
      grasp_i = 0;
    }
    else if(result_i == result_size)
    {
      result_i = 0;
      grasp_i = 0;
    }
    if(grasp_i < 0)
    {
      grasp_i = grasp_size - 1;
    }
    else if(grasp_i == grasp_size)
    {
      grasp_i = 0;
    }
  }
}

void pickGraspInCloud(const vector<Object_recognition::Result>& results, const vector<Object>& objects, const string& scene_path_name)
{
  const int view_width=640, view_height=480;
  result_size = objects.size();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile (scene_path_name, *scene) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", scene_path_name.c_str());
    return;
  }
  CloudT axis_cloud; getAxisCloud(axis_cloud);   
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("grasp visualizer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(scene);
  viewer->addPointCloud<pcl::PointXYZRGB> (scene, color, "sample cloud");
  viewer->initCameraParameters();
  viewer->setSize(view_width, view_height);
  viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  while (!viewer->wasStopped() && !aborted)
  {
    const Object& object = findObject(objects, results[result_i].object_name);
    const vector<Transformation>& grasps = object.getGrasps();
    grasp_size = grasps.size();
    for (int i = 0; i < grasps.size(); ++i) {
      int line_width = (i == grasp_i ? 3 : 1);
      const Transformation& grasp_tf = grasps[i];
      CloudT grasp_cloud;
      pcl::transformPointCloud(axis_cloud, grasp_cloud, grasp_tf);
      vector<Point3d> axis_pts(4);
      for (int i = 0; i < 4; ++i) {
        axis_pts[i] = Point3d(grasp_cloud[i].x, grasp_cloud[i].y, grasp_cloud[i].z);
      }
      pcl::transformPointCloud(grasp_cloud, grasp_cloud, results[result_i].transformation);
      char buffer [10];
      sprintf(buffer, "%i", i);
      string str(buffer);
      string line_name = string("x") + str;
      viewer->addLine<PointT>(grasp_cloud[0], grasp_cloud[1], line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_name.c_str());
      line_name = string("y") + str;
      viewer->addLine<PointT>(grasp_cloud[0], grasp_cloud[2], line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_name.c_str());
      line_name = string("z") + str;
      viewer->addLine<PointT>(grasp_cloud[0], grasp_cloud[3], line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, line_name.c_str());
      viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_name.c_str());
    }
    string selected_msg = "Selected: " + results[result_i].object_name;
    viewer->addText(selected_msg,10, 15, 20, 1.0, 1.0, 0.0);
    viewer->spinOnce(1, true);
    if(grasp_picked)
    {
      Transformation grasp_transformation = results[result_i].transformation * grasps[grasp_i];
      string pick_up_msg = "Grasping: " + results[result_i].object_name;
      cout << pick_up_msg << ", from this pose: " << endl;
      cout << grasp_transformation << endl;
      viewer->addText("Grasping selected object",10, 40, 20, 1.0, 0.0, 0.0);
      viewer->spinOnce();
      break;
    }
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    viewer->removeAllShapes(); // not able to redraw ???
  }
  if(grasp_picked)
  {
    while (!viewer->wasStopped() && !aborted)
    {
      viewer->spinOnce();
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }
  // viewer->saveScreenshot("Grasp image");
  viewer->close();
}

void pickGraspInImage(Mat scene_im, const vector<Object_recognition::Result>& results, const vector<Object>& objects)
{
  string window_name("Grasp picking interface");
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  // setup camera
  cv::Mat camera_matrix = cv::Mat::zeros(3,3,CV_64F);
  camera_matrix.at<double>(0,0) = 525.0;
  camera_matrix.at<double>(0,2) = 319.5;
  camera_matrix.at<double>(1,1) = 525.0;
  camera_matrix.at<double>(1,2) = 239.5;
  camera_matrix.at<double>(2,2) = 1;
  cv::Mat dist_doeffs = cv::Mat::zeros(4,1,CV_64F);
  bool grasp_picked = false, aborted = false;
  int result_i = 0, grasp_i = 0;
  while(!grasp_picked && !aborted)
  {
    // convert transformation to opencv format
    Transformation transformation = results[result_i].transformation;
    cv::Mat_<double> R(3,3,cv::DataType<double>::type);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R.at<double>(i,j) = transformation(i,j);
      }
    }
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Rodrigues(R, rvec);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    for (int i = 0; i < 3; ++i) {
      tvec.at<double>(i,0) = transformation(i,3);
    }

    Mat axis_im = scene_im.clone();
    CloudT axis_cloud; getAxisCloud(axis_cloud);
    const Object& object = findObject(objects, results[result_i].object_name);

    const vector<Transformation>& grasps = object.getGrasps();
    for (int i = 0; i < grasps.size(); ++i)
    {
      int line_width = (i == grasp_i ? 2 : 1);
      const Transformation& grasp_tf = grasps[i];
      CloudT grasp_cloud;
      pcl::transformPointCloud(axis_cloud, grasp_cloud, grasp_tf);
      vector<Point3d> axis_pts(4);
      for (int i = 0; i < 4; ++i) {
        axis_pts[i] = Point3d(grasp_cloud[i].x, grasp_cloud[i].y, grasp_cloud[i].z);
      }
      std::vector<cv::Point2d> projectedAxisPoints;
      cv::projectPoints(axis_pts, rvec, tvec, camera_matrix, dist_doeffs, projectedAxisPoints);
      line(axis_im,projectedAxisPoints[0], projectedAxisPoints[1], Scalar( 0, 0, 255 ), line_width, 8 );
      line(axis_im,projectedAxisPoints[0], projectedAxisPoints[2], Scalar( 0, 255, 0 ), line_width, 8 );
      line(axis_im,projectedAxisPoints[0], projectedAxisPoints[3], Scalar( 255, 0, 0), line_width, 8 );
    }
    string selected_msg = "Selected: " + results[result_i].object_name;
    cv::putText(axis_im, selected_msg, Point(20, axis_im.rows - 20), FONT_HERSHEY_PLAIN, 1.3, Scalar(0,255,255), 2,8);
    imshow(window_name,axis_im);
    char key_stroke = waitKey();
    switch (key_stroke) {
    case CTRL:
      {
      Transformation grasp_transformation = results[result_i].transformation * grasps[grasp_i];
      string pick_up_msg = "Grasping: " + results[result_i].object_name;
      cout << pick_up_msg << ", from this pose: " << endl;
      cout << grasp_transformation << endl;
      cv::putText(axis_im, "Grasping", Point(20, axis_im.rows - 45), FONT_HERSHEY_PLAIN, 1.3, Scalar(0,0,255), 2,8);
      imshow(window_name,axis_im);
      grasp_picked = true;
      }
      break;
    case NUM_NULL:
      aborted = true;
      break;
    case UP_ARROW:
      result_i++;
      grasp_i = 0;
      break;
    case DOWN_ARROW:
      result_i--;
      grasp_i = 0;
      break;
    case RIGHT_ARROW:
      grasp_i++;
      break;
    case LEFT_ARROW:
      grasp_i--;
      break;
    default:
      break;
    }
    // circular iterate through
    if(result_i < 0)
    {
      result_i = results.size() - 1;
    }
    else if(result_i == results.size())
    {
      result_i = 0;
    }
    if(grasp_i < 0)
    {
      grasp_i = grasps.size() - 1;
    }
    else if(grasp_i == grasps.size())
    {
      grasp_i = 0;
    }
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  if(grasp_picked)
  {
    while(cv::waitKey() != NUM_NULL)
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      break;
    }
  }
  destroyWindow(window_name);
  //imwrite("axis_on_vanila.png", axis_im);
}

void readScene (string file_name, pcl::PointCloud<PointT>::Ptr scene)
{
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile (file_name, cloud_blob) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s\n", file_name.c_str());
    return;
  }
  pcl::fromPCLPointCloud2 (cloud_blob, *scene); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
}
