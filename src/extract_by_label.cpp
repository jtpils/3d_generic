// =====================================================================================
//
//       Filename:  extract_by_label.cpp
//
//    Description:
//
//        Version:  1.0
//        Created:  02/27/2014 02:24:36 PM
//       Revision:  none
//       Compiler:  g++
//
//         Author:  destine Lin (), siyuen.lin@gmail.com
//        Company:
//
// =====================================================================================
#include <ros/ros.h>
#include "include/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include "include/custom_passthrough.hpp"
#include <sstream>
#include	<cstdlib>
typedef pcl::PointXYZRGBCamSL  PointT;

  int
main ( int argc, char *argv[] )
{
  if(argc < 3) {ROS_ERROR("no enough params for input."); return(-1);}
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile (argv[1], cloud_blob) == -1)
  {
    ROS_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());

  // Convert to the templated message type
  pcl::fromROSMsg (cloud_blob, *cloud);
  if(pcl::getFieldIndex(cloud_blob, "label") < 0) {ROS_ERROR("no filed named label in the cloud."); return(-1);}

  std::stringstream ss;
  float labelNum;
  ss<<argv[2];  ss>>labelNum;
  pcl::PassThrough<PointT> label_filter;
  label_filter.setInputCloud (cloud);
  label_filter.setFilterFieldName("label");
  label_filter.setFilterLimits(labelNum-0.1, labelNum+0.1);
  label_filter.filter(*cloud_out);
  if(cloud_out->size() == 0) {ROS_ERROR("no label %d for %s.", labelNum, argv[1]); return(-1);}

  std::string fn (argv[3]);
  pcl::io::savePCDFileASCII (fn, *cloud_out);
  ROS_INFO ("Saved %d data points to pcd file.", (int)cloud_out->points.size ());

  return EXIT_SUCCESS;
}				// ----------  end of function main  ----------

