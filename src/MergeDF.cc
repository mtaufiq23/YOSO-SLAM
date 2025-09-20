/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

/* 
 * 2D & 3D info fusion based on Depth filtering ~ MergeDF
 */

#include "MergeDF.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


// Class Constructors
MergeDF::MergeDF(const std::string &strSettingPath)
{
// Statistical Filters
   mStat.setMeanK (30);	     	    // Set to consider the number of neighboring points of the query point when performing statistics. Execute in class initialization
   mStat.setStddevMulThresh (1.0);   // Set the threshold for determining whether it is an outlier point
   //And set the standard deviation multiple to 1. This means that if a point's distance 
   //is more than one standard deviation away from the mean, the point is marked as an outlier and removed.
   // Voxel grid downsampling ====
    mVoxel.setLeafSize( 0.05, 0.05, 0.05);
    mpOD = new ObjectDatabase(strSettingPath);
	removedynamic_cloud = pcl::make_shared<PointCloud>();
}


MergeDF::~MergeDF()
{
    delete mpOD;
}

//Collect object into database
void MergeDF::merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap)
{
	if(!objects.empty())
        { 
          for(unsigned int i=0; i<objects.size(); i++)
          {  
             Cluster cluster; 
             bool ok = mergeOne(objects[i], cluster, depth, pclMap);
             if(ok) 
                   mpOD->addObject(cluster);
          }
        }

}

//Extract object information based on DF
bool MergeDF::mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap)
{
  if(object.prob >0.54)// Prediction probability
  {
	float* depth = (float*)depth_img.data;              //1 channel

	cv::Rect_<float> rect = object.rect;// frame
	int beg = (int)rect.x + ((int)rect.y-1)*depth_img.cols - 1;// 2d bounding box area

// A. Calculate mean of depth in bounding box area
	int count = 0;
	float sum = 0;
	int row_beg = (int)rect.height*0.3;
	int row_end = (int)rect.height*0.7;
	int col_beg = (int)rect.width*0.3;
	int col_end = (int)rect.width*0.7;
	for(int k=row_beg; k<row_end; k++) // Each row
	{   
	  int start = beg + k*depth_img.cols + col_beg; // starting point
	  int end   = beg + k*depth_img.cols + col_end ;// end
	  for(int j = start; j<end; j++)//Each column
	  { 
	    float d = depth[j];
	    if (d < 0.01 || d > 3.0) // This has been converted into meters.
	      continue;
	    sum += d;
	    count++;
	  }
	}
	float depth_threshold = 0.0;
	if(count>0) depth_threshold = sum / count;// Mean Depth
        else 
             return false;

// B. Get the target point cloud index based on the mean depth value
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	row_beg = (int)rect.height*0.2;
	row_end = (int)rect.height*0.8;
	col_beg = (int)rect.width*0.2;
	col_end = (int)rect.width*0.8;
	for(int k=row_beg; k<row_end; k++) // Each row
	{   
	  int start = beg + k*depth_img.cols + col_beg; // starting point
	  int end   = beg + k*depth_img.cols + col_end ;// end
	  for(int j = start; j<end; j++)//Each column
	  { 
	    if( abs(depth[j] - depth_threshold) < 0.2 )// Any object with a depth difference of 0.4m from the mean depth is considered valid
	      inliers->indices.push_back (j); // Record the point cloud index
	  }
	}

        if(inliers->indices.size() < 50)
              return false;

// C. Extract point cloud that contained the object using point cloud index 
        mExtractInd.setInputCloud(pclMap);// Set the input point cloud pointer
	// Setting the index
        mExtractInd.setIndices (inliers);
	PointCloud::Ptr before (new PointCloud);
	mExtractInd.filter (*before);//Extract the point cloud for indexing

// D. Filtering 
	// Voxel grid downsampling
	mVoxel.setInputCloud( before );
	mVoxel.filter( *before );
	// Statistical filtering to remove noise
	PointCloud::Ptr after_stat (new PointCloud);
	mStat.setInputCloud (before);//Set the point cloud to be filtered
	mStat.filter (*after_stat); //Store inliers
        if(after_stat->width * after_stat->height< 30)
              return false;

// E. Calculate the extracted point cloud parameters that contained the detected object
	// Calculate the center of the point cloud
	Eigen::Vector4f cent;
	pcl::compute3DCentroid(*after_stat, cent);
	// Calculate point cloud Point range
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D (*after_stat, minPt, maxPt);

	// Extracted semantic object info
	cluster.object_name = object.object_name;// name
	cluster.class_id    = object.class_id;    // Category ID
	cluster.prob        = object.prob;// Confidence
    cluster.size        = Eigen::Vector3f(maxPt[0]-minPt[0], maxPt[1]-minPt[1], maxPt[2]-minPt[2]);// size
	cluster.centroid    = Eigen::Vector3f(cent[0],  cent[1],  cent[2]); // Center Point~centroid

        return true;
   }
   return false;
}


