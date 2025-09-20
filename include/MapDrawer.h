/*
* This file is a modified version of ORB-SLAM3.<https://github.com/UZ-SLAMLab/ORB_SLAM3>
*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/



#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"
#include<pangolin/pangolin.h>

#include<mutex>
#include <thread>
#include <vtkSMPTools.h>

// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

// pcl*/
#include <pcl/io/pcd_io.h>// Read and write
#include <pcl/common/transforms.h>// Point cloud coordinate transformation
#include <pcl/point_types.h>      // Point type
#include <pcl/filters/voxel_grid.h>// Voxel grid filtering
#include <pcl/filters/passthrough.h>//  Pass filter
#include <pcl/sample_consensus/method_types.h>// Sampling consistency, sampling method
#include <pcl/sample_consensus/model_types.h>// Model
#include <pcl/segmentation/sac_segmentation.h>// Sampling Consistency Segmentation
#include <pcl/filters/extract_indices.h>// Extract point halo index
#include <pcl/filters/statistical_outlier_removal.h>
#include "PointCloude.h"


#include "YoloDetect.h"// 2d target detection results
class YoloDetection;

#include "MergeDF.h"// Fusion of 2D and point cloud information into 3D target database
class MergeDF;



namespace ORB_SLAM3
{

class Settings;


class MapDrawer
{
public:

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings, YoloDetection* pDetector);

	 ~MapDrawer();

    void newParameterLoader(Settings* settings, const string &strSettingPath);

    Atlas* mpAtlas;
    
    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);

	void InsertKeyFrame1(KeyFrame* kf, cv::Mat& imRGB, cv::Mat& imDepth, int idk, vector<KeyFrame*> vpKFs);
  
	queue<uint16_t> omqidk;
	queue<KeyFrame*> omqKeyFrame;
    queue<cv::Mat> omqRGB;
    queue<cv::Mat> omqDepth;
    int omKeyFrameSize;
    uint16_t oKfno = 0;
	std::mutex mMutexOctProc;

	std::thread* omptMapDrawing;

	pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
	pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr vg;

	// Display dense octomap plot
	void DrawOctoMap();

	void DrawGrid();// Show grid by drawing lines

	// Pangolin displays pcl point cloud points===
/*	// Show current frame point cloud*/
	void DrawObs(void);

/*	// Display objects in the database, 3D boxes*/
	void DrawObject();

	// Save octomap map
	void SaveOctoMap(const std::string &filename);

	void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs);

	void ClearOctomap();
	void MapUpdateOctomap();

	vector<KeyFrame*> currentvpKFs;

	void MergeUpdate();
	bool octo_mergebusy = false;
    bool octo_bStop2 = false;
	int octo_mergecount = 0;
	bool lockocto2 = false;

	void LoopUpdate();
	bool octobusy;
    bool octo_loopbusy = false;    
    bool octo_bStop = false;
	int octo_loopcount = 0;
	bool lockocto = false;

	void gndfilter(PointCloud& temp);
	void gndfilter2(PointCloud& temp);

	int octo_lc_enable;
	int octo_m_enable;
	
	bool bIsLocalization;
    bool ot_exist;
	bool vkfHasVal;

	void PlotOctomap();

	bool mdloadedAtlas = false;

	YoloDetection* mpDetector;

	uint16_t mpkfno = 0;
	int matchid = 0;
	int LastOctoNo = 1;
	std::vector<KeyFrame*> mpKeyframes;

	int octodrawobj;

	queue<cv::Mat> omqMask;

	bool isInDynamicRegion(int x,int y,std::vector<cv::Rect_<float> >& vDynamicBorder_);

// Fusion of 2D and point cloud information into 3D target database 
// It needs to be defined as a pointer, otherwise the compiler will report an error
    MergeDF* mpMerge2d3d; 

    vector<PointCloude> pointcloud1;


protected:
     // Generate a point cloud of the current frame, simply filter and separate ground and non-ground
	 PointCloud::Ptr GeneratePointCloudwObj(KeyFrame* kf, cv::Mat& imRGB, cv::Mat& imDepth, std::vector<Object>& objects, std::vector<Object>& dyobjects);
     PointCloud::Ptr GeneratePointCloud1(KeyFrame* kf, cv::Mat& imRGB, cv::Mat& imDepth);
	 
     // Total octomap map inserted, newly generated point cloud
     void InsertScan(octomap::point3d sensorOrigin, 
                     pcl::PointCloud<pcl::PointXYZRGB>& ground, 
                     pcl::PointCloud<pcl::PointXYZRGB>& nonground);
     
	 // speckle
     bool isSpeckleNode(const octomap::OcTreeKey &nKey);
     
     // update octomap
     void UpdateOctomap1(KeyFrame* kf);
     
     // Highly generated color
     void heightMapColor(double h, double& r, double &g, double& b);
     
	condition_variable keyFrameUpdated;
    mutex keyFrameUpdateMutex;
    mutex keyframeMutex;

	PointCloud  true_ground; // Ground point cloud
    PointCloud  true_nonground;// Non-ground point cloud
	PointCloud  true_temp;

	

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};


private:

   // int last_obj_size;

    pcl::PointCloud<pcl::PointXYZRGB> observation; // Current frame point cloud===

    uint16_t  lastKeyframeSize =0;// Number of last keyframes
    
    int oid = 0;
    vector<octomap::ColorOcTree> *OctreeAtlas;
    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;// octomap graph accuracy
	double octo_meank;
    double octo_thresh;
	int plotKFpcl;	// bool to trigger kf pcl plotter 
	int plotKFpcl_static_only;	// bool to trigger kf pcl plotter with filtered dynamic objects
	int plotKFpcl_dynamic_only;	// bool to trigger kf pcl plotter with filtered static objects
	double set_leaf_size;	//voxel grid size
	double occ_thresh; // The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
    double OctoClampingThresMin;
	double OctoClampingThresMax;
	double OctoSetProbHit;
	double OctoSetProbMiss;
	int level; // Octree allowable map depth/level
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;

	double MAX_POINTCLOUD_DEPTH;
 	double MIN_POINTCLOUD_DEPTH;
	double MAX_VERTICAL_RANGE;
	double MIN_VERTICAL_RANGE;

	//Bool to trigger filter ground data or not from pcl
	int filter_ground;

    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }

};

} //namespace ORB_SLAM3

#endif // MAPDRAWER_H
