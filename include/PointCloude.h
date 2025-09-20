/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

/*
* Class for storing pointclouds 
*/


#ifndef POINTCLOUDE_H
#define POINTCLOUDE_H

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM3
{

class PointCloude
{
    typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
public:
	PointCloude();
    PointCloud::Ptr pcE;
	PointCloud pcO;
public:
    Eigen::Isometry3d T;
    int pcID; 
//protected:    
   
};


} //namespace ORB_SLAM3

#endif // POINTCLOUDE_H
