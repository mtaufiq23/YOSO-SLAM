/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

/* 
 * 2D & 3D info fusion based on Depth filtering ~ MergeDF
 */

#ifndef MERGEDF_H
#define MERGEDF_H



#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h> // Voxel Grid Filtering
#include <pcl/filters/statistical_outlier_removal.h>// Statistical Filters
#include <pcl/filters/extract_indices.h>            // Extract the corresponding point cloud according to the point cloud index
#include <pcl/filters/filter.h>                     // Remove the nan points and make them unordered
#include <pcl/common/centroid.h>

#include <Eigen/Core>

#include <vector>

#include "ObjectDatabase.h"// Target database
#include "YoloDetect.h"      // 2D detector

struct Cluster;

class ObjectDatabase;

class MergeDF
{

public:
    typedef pcl::PointXYZRGB PointT;// Point type xyzrgb point+color
    typedef pcl::PointCloud<PointT> PointCloud;// Point cloud type
    MergeDF(const std::string &strSettingPath);
    ~MergeDF();
    void merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap);

    ObjectDatabase* mpOD;// Need to be defined as a pointer <> Public to facilitate visualization 

	PointCloud::Ptr removedynamic_cloud;

protected:
    bool mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap);

    pcl::ExtractIndices<PointT> mExtractInd;// Index Extract Point Cloud Analyzer Set as a member object
    pcl::VoxelGrid<PointT>  mVoxel;             // Voxel filter object   
    pcl::StatisticalOutlierRemoval<PointT> mStat; // Statistical filtering to remove outliers   
};

#endif // MERGEDF_H
