// =====================================================================================
//
//       Filename:  labeller.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  03/15/2014 08:06:56 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include "include/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>

typedef pcl::PointXYZRGBCamSL  PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//----------------------------------------------------------------------
//  Global variables list
//----------------------------------------------------------------------
pcl::visualization::PCLVisualizer viewer("3D Viewer");
PointCloudT::Ptr cloud_ptr(new PointCloudT);
PointCloudT::Ptr new_cloud_ptr (new PointCloudT);
std::string pressed_num;
std::vector<int> selected;
std::string infile;
uint8_t r=255, g=0, b=0;
uint32_t seg_color = ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);

// ===  FUNCTION  ======================================================================
//         Name:  markLabel
//  Description:
// =====================================================================================
void markLabel(std::vector<int> &indices,  int label_num)
{
  for(size_t i = 0; i < indices.size(); i++)
    cloud_ptr->points[indices[i]].label = label_num;
}

// ===  FUNCTION  ======================================================================
//         Name:  keyboardEventOccurred
//  Description:
// =====================================================================================
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{
  if(event.isCtrlPressed() && event.keyUp() && !pressed_num.empty()) {
    int label_num = boost::lexical_cast<int>(pressed_num);
    std::cout << "segment will be marked as label " << label_num << "." << std::endl;
    markLabel(selected, label_num);
    pressed_num.clear();
  }else {
    char word = event.getKeyCode();
    if(event.keyUp() && word >= '0' && word <= '9')
      pressed_num += word;
    else if (event.keyUp() && word == 's') {
      std::string fn = "labelled_" + infile;
      pcl::io::savePCDFile(fn, *cloud_ptr);
      std::cout << "point cloud has saved into " << fn.c_str() << "." << std::endl;
    }
  }
}

// ===  FUNCTION  ======================================================================
//         Name:  selectionAreaOccurred
//  Description:
// =====================================================================================
void selectionAreaOccurred(const pcl::visualization::AreaPickingEvent &event)
{
  if(!event.getPointsIndices (selected)) return;
  viewer.removePointCloud("sample cloud");
  *new_cloud_ptr = *cloud_ptr;

  std::cout << selected.size() << " of points being selected" << std::endl;
  for(size_t i=0; i < selected.size(); i++)
    new_cloud_ptr->points[selected[i]].rgb = *reinterpret_cast<float*>(&seg_color);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(new_cloud_ptr);
  viewer.addPointCloud<PointT> (new_cloud_ptr, rgb, "sample cloud");
}

// ===  FUNCTION  ======================================================================
//         Name:  colorVisual
//  Description:
// =====================================================================================
void colorVisualInWindow(PointCloudT::ConstPtr cloud)
{
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.initCameraParameters();
  viewer.registerKeyboardCallback(keyboardEventOccurred);
  viewer.registerAreaPickingCallback(selectionAreaOccurred);
}
  int
main ( int argc, char *argv[] )
{
  // Load in point cloud
  if(argc < 2) std::cout << "Usage: test.pcd" << std::endl;
  infile = argv[1];
  if(pcl::io::loadPCDFile<PointT>(infile, *cloud_ptr) != 0) {
    std::cout << "can not load into " << infile.c_str() << "." << std::endl;
    exit(-1);
  }
  std::cout << cloud_ptr->size() << " point cloud has been loaded into mem." << std::endl;

  colorVisualInWindow(cloud_ptr);

  // Main loop
  viewer.spin();
  return 0;
}				// ----------  end of function main  ----------
