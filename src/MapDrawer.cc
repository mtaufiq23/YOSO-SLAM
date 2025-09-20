/*
* This file is a modified version of ORB-SLAM3.<https://github.com/UZ-SLAMLab/ORB_SLAM3>
*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <thread>
#include <boost/filesystem.hpp>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace pangolin{
extern "C" const unsigned char AnonymousPro_ttf[];
}

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings, YoloDetection *pDetector):
    mpDetector(pDetector),
	mpAtlas(pAtlas),
    m_octree(NULL),
    m_maxRange(-1.0),
    m_useHeightMap(true),
    m_res(0.05),
    m_colorFactor(0.8),
    m_treeDepth(0),
    m_maxTreeDepth(0), 
	ot_exist(false),
    vkfHasVal(false)

{
    if(settings){
        newParameterLoader(settings, strSettingPath);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

	m_octree = new octomap::ColorOcTree(m_res);// octomap graph accuracy
    // initialize octomap
    m_octree->setClampingThresMin(OctoClampingThresMin); // These parameters can be passed in ===
    m_octree->setClampingThresMax(OctoClampingThresMax);
    m_octree->setProbHit(OctoSetProbHit);
    m_octree->setProbMiss(OctoSetProbMiss);

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;

	occ_thresh = occ_thresh; // The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
    level = level; // Octree allowable map depth/level

	//Statistical filter settings========
	statistical_filter.setMeanK(octo_meank);
    statistical_filter.setStddevMulThresh(octo_thresh);

	// voxel grid filtering settings======
    vg = pcl::make_shared<pcl::VoxelGrid<pcl::PointXYZRGB>>();
    vg->setLeafSize(set_leaf_size, set_leaf_size, set_leaf_size);// Voxel grid size	

    bIsLocalization = false;
	ot_exist = false;
	oid++;
	std::cout<< "日 Created new octomap, id: " << oid << ". Total octomap in current run time: " << oid << std::endl;
	
//	bIsLocalization = false;
    
	 mpMerge2d3d = new MergeDF(strSettingPath);
}


MapDrawer::~MapDrawer()
{
   delete mpMerge2d3d;
   delete m_octree;
}


void MapDrawer::newParameterLoader(Settings *settings, const string &strSettingPath) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();

	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	m_res = fSettings["OctoMap.Resolution"];// octomap resolution
	octo_meank = fSettings["OctoMap.meank"];   //Statistical filter mean
    octo_thresh = fSettings["OctoMap.thresh"];	////Statistical filter sd
	plotKFpcl = fSettings["OctoMap.PlotKFpcl"];// bool to trigger kf pcl plotter
	set_leaf_size = fSettings["OctoMap.SetLeafSize"];// voxel grid size
	occ_thresh = fSettings["OctoMap.ProbabilityTh"];// The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
	OctoClampingThresMin = fSettings["OctoMap.SetClampingThresMin"]; 
    OctoClampingThresMax = fSettings["OctoMap.SetClampingThresMax"];
    OctoSetProbHit = fSettings["OctoMap.SetProbHit"];
    OctoSetProbMiss = fSettings["OctoMap.SetProbMiss"];
    level = fSettings["OctoMap.SetDepthLevel"]; // Octree allowable map depth/level
	MAX_POINTCLOUD_DEPTH = fSettings["OctoMap.MaxPclDepthRange"];
	MIN_POINTCLOUD_DEPTH = fSettings["OctoMap.MinPclDepthRange"];
	MAX_VERTICAL_RANGE = fSettings["OctoMap.MaxVerticalRange"];
	MIN_VERTICAL_RANGE = fSettings["OctoMap.MinVerticalRange"];
	filter_ground = fSettings["OctoMap.FilterGndData"];//Bool to trigger filter ground data or not from pcl
    octo_lc_enable = fSettings["OctoMap.EnableLoopClosing"]; //Bool to trigger loop closing 
	octo_m_enable = fSettings["OctoMap.EnableMapMerging"]; //Bool to trigger map merging
	octodrawobj = fSettings["OctoMap.DrawObjects"];	//bool to trigger draw detected static objects 
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }


	m_res = fSettings["octoMap.VoxelGridResolution"];// octomap graph accuracy
	octo_meank = fSettings["octoMap.meank"];   //Statistical filter mean
    octo_thresh = fSettings["octoMap.thresh"];	////Statistical filter sd
	plotKFpcl = fSettings["octoMap.PlotKFpcl"];// bool to trigger kf pcl plotter
	plotKFpcl_static_only = fSettings["OctoMap.PlotKFpclStaticOnly"];// bool to trigger kf pcl plotter with filtered dynamic objects
	plotKFpcl_dynamic_only = fSettings["OctoMap.PlotKFpclDynamicOnly"];// bool to trigger kf pcl plotter with filtered static objects
	set_leaf_size = fSettings["octoMap.SetLeafsize"];// voxel grid size
	occ_thresh = fSettings["octoMap.ProbabilityTh"];// The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
	OctoClampingThresMin = fSettings["OctoMap.SetClampingThresMin"]; 
    OctoClampingThresMax = fSettings["OctoMap.SetClampingThresMax"];
    OctoSetProbHit = fSettings["OctoMap.SetProbHit"];
    OctoSetProbMiss = fSettings["OctoMap.SetProbMiss"];
	level = fSettings["octoMap.SetDepthLevel"]; // Octree allowable map depth/level
	MAX_POINTCLOUD_DEPTH = fSettings["octoMap.MaxPclDepthRange"];
	MIN_POINTCLOUD_DEPTH = fSettings["octoMap.MinPclDepthRange"];
	MAX_VERTICAL_RANGE = fSettings["octoMap.MaxVerticalRange"];
	MIN_VERTICAL_RANGE = fSettings["octoMap.MinVerticalRange"];
	filter_ground = fSettings["octoMap.FilterGndData"];//Bool to trigger filter ground data or not from pcl
	octo_lc_enable = fSettings["octoMap.EnableLoopClosing"]; //Bool to trigger loop closing 
	octo_m_enable = fSettings["octoMap.EnableMapMerging"]; //Bool to trigger map merging
	octodrawobj = fSettings["octoMap.DrawObjects"]; //bool to trigger draw detected static objects

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,1.0f,0.6f);	//purple
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
//    glColor3f(1.0f,0.5f,0.0f); //orange
	glColor3f(1.0f,0.0f,1.0f); //Purple
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}



// Show grid by drawing lines ======
void MapDrawer::DrawGrid()
{
    glBegin(GL_LINES);// draw line ======
    glLineWidth(1);       // Line width======

    glColor3f(0.5,0.5,0.5); //gray  gray lines
    int size =10;
    for(int i = -size; i <= size ; i++){
// xz plane vertical x-axis straight line x-axis -10,-9,...,0,1,2,...,10 21Line, z-direction range， -10～10
      glVertex3f(i,0.6,  size);
      glVertex3f(i, 0.6, -size);

// xz Plane vertical z-axis straight line z-axis -10,-9,...,0,1,2,...,10  21Line, x-direction range， -10～10
      glVertex3f( size, 0.6, i);
      glVertex3f(-size, 0.6, i);
    }
    glEnd();
}



void MapDrawer::ClearOctomap()
{	
    oid++;
	delete m_octree;
	m_octree = new octomap::ColorOcTree(m_res);// octomap graph accuracy
    // initialize octomap
    m_octree->setClampingThresMin(OctoClampingThresMin); // These parameters can be passed in ===
    m_octree->setClampingThresMax(OctoClampingThresMax);
    m_octree->setProbHit(OctoSetProbHit);
    m_octree->setProbMiss(OctoSetProbMiss);

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;

	occ_thresh = occ_thresh; // The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
    level = level; // Octree allowable map depth/level
	std::cout<< "日 Created new octomap, id: " << oid << ". Total octomap in current run time: " << oid << std::endl;
}


void MapDrawer::MapUpdateOctomap()
{
//	m_octree = OctreeAtlas[oid];
}


//Do ground filter during received keyframe period
void MapDrawer::InsertKeyFrame1(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth, int idk, vector<KeyFrame*> vpKFs) 
{
	cout<<"日 octomap received a keyframe, id = "<<idk<<" No."<<kf->mnId<<endl;
    cout<<"日 octomap vpKFs: "<<vpKFs.size()<<endl;

    std::lock_guard<std::mutex> lck_loadKF(mMutexOctProc);
	currentvpKFs = vpKFs;

    omqKeyFrame.push(kf);
    omqRGB.push(imRGB.clone());
    omqDepth.push(imDepth.clone());	

	PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );//Get keyframe pose

    if (octodrawobj == 1)
	{
		for (int i=0;i<currentvpKFs.size();i++)
		{            
			if(currentvpKFs[i]->mnId == mpkfno)
			{
				matchid = i;
				cout<<"日 Match mnID: currentvpKFs "<<matchid<<endl;
				break;
			}
		}   
		
		if(currentvpKFs[matchid]->mvObject.size()>0 || currentvpKFs[matchid]->mvDyObject.size()>0)
		{
			//iterate over keyframe point clouds
			pointcloude.pcE = GeneratePointCloudwObj(kf, imRGB, imDepth, currentvpKFs[matchid]->mvObject, currentvpKFs[matchid]->mvDyObject);
		}
		else
		{
			//Iterate over keyframe point clouds
			pointcloude.pcE = GeneratePointCloud1(kf, imRGB, imDepth);
		}
	}
    else
	{
		pointcloude.pcE = GeneratePointCloud1(kf, imRGB, imDepth);//Iterate over keyframe point clouds
	}
	
	pointcloud1.push_back(pointcloude);
	mpkfno++;
}


void MapDrawer::PlotOctomap()
{
	// octomap with color
    octomap::ColorOcTree::tree_iterator it  = m_octree->begin_tree();
    octomap::ColorOcTree::tree_iterator end = m_octree->end_tree();
    int counter = 0;// count
//    double occ_thresh = 0.8; // The probability threshold is originally 0.9. The larger the value, the fewer octomap grids are displayed.
//    int level = 16; // Octree map depth???
    glClearColor(1.0f,1.0f,1.0f,1.0f);// color + transparency

    glDisable(GL_LIGHTING);
    glEnable (GL_BLEND);

    ////DRAW OCTOMAP BEGIN//////
    // double stretch_factor = 128/(1 - occ_thresh); //1280.0
// occupancy range in which the displayed cubes can be

    for(; it != end; ++counter, ++it)
    {
		
        if(level != it.getDepth())
        {
            continue;
        }

        double occ = it->getOccupancy();//Possession probability=================
  
        if(occ < occ_thresh) // Low probability of possession ==== Not displayed
        {
//			m_octree->deleteNode(it.getKey(), level);// free the node====
            continue;
        }        

        double minX, minY, minZ, maxX, maxY, maxZ;
        m_octree->getMetricMin(minX, minY, minZ);
        m_octree->getMetricMax(maxX, maxY, maxZ);

       float halfsize = it.getSize()/2.0;// half size
       float x = it.getX();
       float y = it.getY();
       float z = it.getZ();

// Height
	   double h;
	   if (minY >= maxY) h = 0.5;
	   else h = ( std::min(std::max((y-minY)/(maxY-minY), 0.0), 1.0))*0.8;


// Calculate color by height
       double r, g, b;
       heightMapColor(h, r,g,b);

       glBegin(GL_TRIANGLES); // 
       //Front
       glColor3d(r, g, b);// Display color=====
       glVertex3f(x-halfsize,y-halfsize,z-halfsize);// - - - 1
       glVertex3f(x-halfsize,y+halfsize,z-halfsize);// - + - 2
       glVertex3f(x+halfsize,y+halfsize,z-halfsize);// + + -3

       glVertex3f(x-halfsize,y-halfsize,z-halfsize); // - - -
       glVertex3f(x+halfsize,y+halfsize,z-halfsize); // + + -
       glVertex3f(x+halfsize,y-halfsize,z-halfsize); // + - -4

       //Back
       glVertex3f(x-halfsize,y-halfsize,z+halfsize); // - - + 1
       glVertex3f(x+halfsize,y-halfsize,z+halfsize); // + - + 2
       glVertex3f(x+halfsize,y+halfsize,z+halfsize); // + + + 3

       glVertex3f(x-halfsize,y-halfsize,z+halfsize); // - - +
       glVertex3f(x+halfsize,y+halfsize,z+halfsize); // + + +
       glVertex3f(x-halfsize,y+halfsize,z+halfsize); // - + + 4

       //Left
       glVertex3f(x-halfsize,y-halfsize,z-halfsize); // - - - 1
       glVertex3f(x-halfsize,y-halfsize,z+halfsize); // - - + 2
       glVertex3f(x-halfsize,y+halfsize,z+halfsize); // - + + 3

       glVertex3f(x-halfsize,y-halfsize,z-halfsize); // - - -
       glVertex3f(x-halfsize,y+halfsize,z+halfsize); // - + +
       glVertex3f(x-halfsize,y+halfsize,z-halfsize); // - + - 4

       //Right
       glVertex3f(x+halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y+halfsize,z-halfsize);
       glVertex3f(x+halfsize,y+halfsize,z+halfsize);

       glVertex3f(x+halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y+halfsize,z+halfsize);
       glVertex3f(x+halfsize,y-halfsize,z+halfsize);

       //top
       glVertex3f(x-halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y-halfsize,z+halfsize);

       glVertex3f(x-halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y-halfsize,z+halfsize);
       glVertex3f(x-halfsize,y-halfsize,z+halfsize);

       //bottom
       glVertex3f(x-halfsize,y+halfsize,z-halfsize);
       glVertex3f(x-halfsize,y+halfsize,z+halfsize);
       glVertex3f(x+halfsize,y+halfsize,z+halfsize);

       glVertex3f(x-halfsize,y+halfsize,z-halfsize);
       glVertex3f(x+halfsize,y+halfsize,z+halfsize);
       glVertex3f(x+halfsize,y+halfsize,z-halfsize);
       glEnd();

       glBegin(GL_LINES); // line segment
       glColor3f(0,0,0);
       //
       glVertex3f(x-halfsize,y-halfsize,z-halfsize);// - - - 1
       glVertex3f(x-halfsize,y+halfsize,z-halfsize);

       glVertex3f(x-halfsize,y+halfsize,z-halfsize);// - + - 2
       glVertex3f(x+halfsize,y+halfsize,z-halfsize);// + + -3

       glVertex3f(x+halfsize,y+halfsize,z-halfsize);// + + -3
       glVertex3f(x+halfsize,y-halfsize,z-halfsize); // + - -4

       glVertex3f(x+halfsize,y-halfsize,z-halfsize); // + - -4
       glVertex3f(x-halfsize,y-halfsize,z-halfsize);// - - - 1


       // back

       glVertex3f(x-halfsize,y-halfsize,z+halfsize); // - - + 1
       glVertex3f(x+halfsize,y-halfsize,z+halfsize); // + - + 2

       glVertex3f(x+halfsize,y-halfsize,z+halfsize); // + - + 2
       glVertex3f(x+halfsize,y+halfsize,z+halfsize); // + + + 3

       glVertex3f(x+halfsize,y+halfsize,z+halfsize); // + + + 3
       glVertex3f(x-halfsize,y+halfsize,z+halfsize); // - + + 4

       glVertex3f(x-halfsize,y+halfsize,z+halfsize); // - + + 4
       glVertex3f(x-halfsize,y-halfsize,z+halfsize); // - - + 1

       // top
       glVertex3f(x+halfsize,y-halfsize,z-halfsize);
       glVertex3f(x+halfsize,y-halfsize,z+halfsize);

       glVertex3f(x-halfsize,y-halfsize,z+halfsize);
       glVertex3f(x-halfsize,y-halfsize,z-halfsize);

        // bottom

       glVertex3f(x-halfsize,y+halfsize,z+halfsize);
       glVertex3f(x+halfsize,y+halfsize,z+halfsize);

       glVertex3f(x-halfsize,y+halfsize,z-halfsize);
       glVertex3f(x+halfsize,y+halfsize,z-halfsize);
       glEnd();
    }
}


// Draw octomap 
void MapDrawer::DrawOctoMap()
{

	{
       std::lock_guard<std::mutex> lck_loadKFSize(mMutexOctProc);
       omKeyFrameSize = omqKeyFrame.size();			
    }

    int N = currentvpKFs.size();// Current number of keyframes

	if(N>0) 
	{
		ot_exist = false;
		vkfHasVal = true;
	}

    if(N==0 && !ot_exist) // The system starts with no keyframes
    {
        	m_octree->clear();// Clear octree map if no initial offline map being loaded.
		    lastKeyframeSize = 0;
			vkfHasVal = false;
		    return;
    }
      
	if(!bIsLocalization && !ot_exist) 
	{
		if(octo_loopbusy || octo_bStop)
    	{
//			cout<<"日 Octomap loop closing process currently running. Wait for it to finish..." <<endl;
			cout<< " " <<endl;
			return;
    	}
		if(octo_mergebusy || octo_bStop2)
    	{
//        	cout<<"日 Octomap merging process currently running. Wait for it to finish..." <<endl;
			cout<< " " <<endl;
			return;
    	}
		
		octobusy = true;
		UpdateOctomap1(omqKeyFrame.front());	// update octomap per kf while doing gnd filter as soon as kf received

		PlotOctomap();
		octobusy = false;	

		if (plotKFpcl) DrawObs(); // Draw the current frame point cloud 
    }
	else PlotOctomap();	//plot fixed octree during localization mode	 
}



// Update octomap per keyframe //Do ground filter during received keyframe period
void MapDrawer::UpdateOctomap1(KeyFrame *kf)
{
  int N = omKeyFrameSize;

  if (N>1)
  {
        oKfno+=1; 
		std::cout<< std::endl << "() Draw Octomap KeyFrame: "<< oKfno << std::endl;

		Eigen::Isometry3d pose = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose());

	    omqKeyFrame.pop();
		omqRGB.pop();
	    omqDepth.pop();

        octomap::point3d sensorOrigin = octomap::point3d( pose(0,3), pose(1,3), pose(2,3));// point cloud origin

        InsertScan(sensorOrigin, true_ground, true_nonground);// Insert new point cloud into octomap map

        lastKeyframeSize = N-1;// Update the number of keyframes that have been processed
  }
}


// Draw octomap colors by height======
void MapDrawer::heightMapColor(double h, 
                               double& r, 
                               double &g, 
                               double& b)
{

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;

    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;

    if(!(i & 1))
    {
        f = 1 - f;
    }
    m = v * (1-s);
    n = v * (1- s*f);

    switch(i)
    {
        case 6:
        case 0:
            r = v; g = n; b = m;
            break;
        case 1:
            r = n; g = v; b = m;
            break;
        case 2:
            r = m; g = v; b = n;
            break;
        case 3:
            r = m; g = n; b = v;
            break;
        case 4:
            r = n; g = m; b = v;
            break;
        case 5:
            r = v; g = m; b = n;
            break;
        default:
            r = 1; g = 0.5; b = 0.5;
         break;

    }

}


// Display the target object in the database in Pangolin~viewer
void MapDrawer::DrawObject()
{
  if (LastOctoNo != oid) 
  {
	mpMerge2d3d->mpOD->mClusters.clear();
	std::cout<< "Octomap has changed!" << std::endl;
	std::cout<< "Clear object database..." << std::endl;
  }

  std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;
  int objnumber = Clusters.size(); // Target database size
 
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  pangolin::GlFont *myfont = new pangolin::GlFont(pangolin::AnonymousPro_ttf, 20);
  
  if( objnumber > 0)
  { 
    //std::cout<< "OD size: " << objnumber << std::endl;
    for(int m=0; m<objnumber; m++)
    {
      Cluster & cluster = Clusters[m];// a target object
      Eigen::Vector3f size  = cluster.size;     // size
      Eigen::Vector3f cent  = cluster.centroid; //center point

      // Cuboid, 8 vertices, 12 line segments
      glBegin(GL_LINES); // line segment=======

      cv::Scalar color =  mpMerge2d3d->mpOD->getObjectColor(cluster.class_id); // Defined object color
      glColor3f(color.val[0]/255.0, color.val[1]/255.0, color.val[2]/255.0);

      // 12 line segments =============================================
       // Above 4
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//

       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
       // In between 4
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]+size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
       // Bottom 4
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//
       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]-size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//

       glVertex3f(cent[0]+size[0]/2.0,cent[1]+size[1]/2.0,cent[2]-size[2]/2.0);//
       glVertex3f(cent[0]+size[0]/2.0,cent[1]-size[1]/2.0,cent[2]-size[2]/2.0);//

      glEnd();

	  glColor3f(0.0, 0.0, 0.0);
      std::string str1 = cluster.object_name;
      std::string str2 = to_string(cluster.prob);
      std::string str =  str1 + "(" + str2 + ")";
//	  myfont->Text(str.c_str()).Draw(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);
      pangolin::GlFont::I().Text(str.c_str()).Draw(cent[0]-size[0]/2.0,cent[1]-size[1]/2.0,cent[2]+size[2]/2.0);
    }
  }

 // mlast_obj_size = objnumber;
 LastOctoNo = oid;
}


//Generate a point cloud with obj of the current frame thats contains objects, simply filter and separate ground and non-ground 
PointCloud::Ptr MapDrawer::GeneratePointCloudwObj(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth, std::vector<Object>& objects, std::vector<Object>& dyobjects)
{
    PointCloud::Ptr cloud (new PointCloud);
    PointCloud::Ptr cloud_temp(new PointCloud);
	PointCloud::Ptr cloud_unfiltered(new PointCloud);

	cloud->resize(imDepth.rows * imDepth.cols);
    cloud->width    =  imDepth.cols;  
    cloud->height   =  imDepth.rows;// Ordered point cloud
    cloud->is_dense =  false;// Non-dense point clouds will have bad points and may contain values ​​such as inf/NaN.

    cloud_unfiltered->resize(imDepth.rows * imDepth.cols);
    cloud_unfiltered->width    =  imDepth.cols;  
	cloud_unfiltered->height   =  imDepth.rows;// Ordered point cloud
	cloud_unfiltered->is_dense =  false;// Non-dense point clouds will have bad points and may contain values ​​such as inf/NaN.


    for ( int m=0; m<(imDepth.rows); m+=1 )// each line
     {
          for ( int n=0; n<(imDepth.cols); n+=1 )//each column
          {
		      bool IsDynamic = false;
			  float d = imDepth.ptr<float>(m)[n];// The unit of depth is m. Points within 0~3m are retained.
		      if (d < MIN_POINTCLOUD_DEPTH || d > MAX_POINTCLOUD_DEPTH || isnan(d)) continue;	// D435i Camera measurement range 0.01～3m

			//Filter out all points inside the dynamic area          
			  if (isInDynamicRegion(n,m,kf->mvPotentialDynamicBorderForMapping)) IsDynamic = true;	

			  float y = ( m - kf->cy) * d / kf->fy;
		      if(y < MIN_VERTICAL_RANGE || y > MAX_VERTICAL_RANGE) continue;// Reserve points within the vertical direction ex. -3~3m
			  int ind = m * imDepth.cols + n;// total index.

		        if (!IsDynamic)				//Thus, only add those which are static points to the point cloud
				{					 
					  if (d == 0) cloud->points[ind].z = NAN;
				      else cloud->points[ind].z = d;
				      cloud->points[ind].x = ( n - kf->cx) * d / kf->fx;
				      cloud->points[ind].y = y;
					  //Color map ~ calculates point cloud color
				      cloud->points[ind].b = kf->mImRGB.ptr<cv::Vec3b>(m)[n][0];
				      cloud->points[ind].g = kf->mImRGB.ptr<cv::Vec3b>(m)[n][1];
				      cloud->points[ind].r = kf->mImRGB.ptr<cv::Vec3b>(m)[n][2];
				}

			    if (d == 0) cloud_unfiltered->points[ind].z = NAN;
				else cloud_unfiltered->points[ind].z = d;
				cloud_unfiltered->points[ind].x = ( n - kf->cx) * d / kf->fx;
				cloud_unfiltered->points[ind].y = y;
			    //Color map ~ calculates point cloud color
				cloud_unfiltered->points[ind].b = kf->mImRGB.ptr<cv::Vec3b>(m)[n][0];
				cloud_unfiltered->points[ind].g = kf->mImRGB.ptr<cv::Vec3b>(m)[n][1];
				cloud_unfiltered->points[ind].r = kf->mImRGB.ptr<cv::Vec3b>(m)[n][2];

          }
     }

	//Store unprocessed pcl
	pcl::copyPointCloud( *cloud, *cloud_temp);

	// Convert to world coordinates====
    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr temp(new PointCloud);
    pcl::transformPointCloud( *cloud, *temp, T.inverse().matrix());
    
	// 3D target information acquisition=====	
	PointCloud::Ptr temp_unfiltereds(new PointCloud);
	pcl::transformPointCloud( *cloud_unfiltered, *temp_unfiltereds, T.inverse().matrix());
	mpMerge2d3d->merge(objects, imDepth, temp_unfiltereds);
    
	std::vector<Cluster>& Clusters = mpMerge2d3d->mpOD->mClusters;
    int objnumber = Clusters.size(); // Target database size
    std::cout<< "OD size: " << objnumber << std::endl;
    for( int m=0; m<objnumber; m++)
    {
      Cluster & cluster = Clusters[m];// a target object
      Eigen::Vector3f size  = cluster.size;     // size
      Eigen::Vector3f cent  = cluster.centroid; //center point

      std::cout<< "Obj: " << cluster.object_name << " " << cluster.prob << " "
               << cent[0] << " " << cent[1] << " " << cent[2] << " "
               << size[0] << " " << size[1] << " " << size[2] << " "
               << std::endl;
    }

	// voxel grid filtering======
    vg->setInputCloud(temp);
	vg->filter(*temp);
	
	PointCloud _temp;
	pcl::copyPointCloud(*temp, _temp);
	gndfilter(_temp);

	if (plotKFpcl)
	{		
		PointCloud temp_unfiltered;
		vg->setInputCloud(temp_unfiltereds);
		vg->filter(*temp_unfiltereds);	
		pcl::copyPointCloud(*temp_unfiltereds, temp_unfiltered);
		RegisterObs(temp_unfiltered);
	}

	return temp;
}


// Generate a point cloud of the current frame, simply filter and separate ground and non-ground
PointCloud::Ptr MapDrawer::GeneratePointCloud1(KeyFrame *kf, cv::Mat &imRGB, cv::Mat &imDepth)
{
    PointCloud::Ptr cloud (new PointCloud);
    PointCloud::Ptr cloud_temp(new PointCloud);
	PointCloud::Ptr cloud_unfiltered(new PointCloud);
	
    for ( int m=0; m<(imDepth.rows); m+=1 )// each line
     {
          for ( int n=0; n<(imDepth.cols); n+=1 )//each column
          {
		      bool IsDynamic = false;
			  float d = imDepth.ptr<float>(m)[n];// The unit of depth is m. Points within 0~3m are retained.
		      if (d < MIN_POINTCLOUD_DEPTH || d > MAX_POINTCLOUD_DEPTH || isnan(d)) continue;	// D435i Camera measurement range 0.01～3m

			//Filter out all points inside the dynamic area          
			  if (isInDynamicRegion(n,m,kf->mvPotentialDynamicBorderForMapping)) IsDynamic = true;	

			  float y = ( m - kf->cy) * d / kf->fy;
			  if(y < MIN_VERTICAL_RANGE || y > MAX_VERTICAL_RANGE) continue;// Reserve points within the vertical direction ex. -3~3m
			  pcl::PointXYZRGB p;

		        if (!IsDynamic)				//Thus, only add those which are static points to the point cloud
				{	
					  
				      p.z = d;								//Camera model, only calculate point clouds of key frames
				      p.x = ( n - kf->cx) * d / kf->fx;	//The coordinate system is opposite to the pcl coordinate system, so p.z=-d 
				      p.y = y;

					  //Color map ~ calculates point cloud color
				      p.b = kf->mImRGB.ptr<cv::Vec3b>(m)[n][0];	
				      p.g = kf->mImRGB.ptr<cv::Vec3b>(m)[n][1];
				      p.r = kf->mImRGB.ptr<cv::Vec3b>(m)[n][2];
				      cloud->push_back(p);	
				}

				if (plotKFpcl)
				{
					  p.z = d;								//Camera model, only calculate point clouds of key frames
				      p.x = ( n - kf->cx) * d / kf->fx;	//The coordinate system is opposite to the pcl coordinate system, so p.z=-d 
				      p.y = y;

					  //Color map ~ calculates point cloud color
				      p.b = kf->mImRGB.ptr<cv::Vec3b>(m)[n][0];	
				      p.g = kf->mImRGB.ptr<cv::Vec3b>(m)[n][1];
				      p.r = kf->mImRGB.ptr<cv::Vec3b>(m)[n][2];
				      cloud_unfiltered->push_back(p);
				}
          }
     }

	//Store unprocessed pcl
	pcl::copyPointCloud( *cloud, *cloud_temp);
    cloud->is_dense = false;
	cloud_unfiltered->is_dense = false;
	
// Convert to world coordinates====
    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );

	PointCloud::Ptr temp (new PointCloud);
	pcl::transformPointCloud( *cloud, *temp, T.inverse().matrix());

	// voxel grid filtering======
    vg->setInputCloud(temp);
	vg->filter(*temp);

	PointCloud _temp;
	pcl::copyPointCloud(*temp, _temp);
	gndfilter(_temp);

	if (plotKFpcl)
	{			
		PointCloud temp_unfiltered;
		vg->setInputCloud(cloud_unfiltered);
		vg->filter(*cloud_unfiltered);	
		pcl::transformPointCloud( *cloud_unfiltered, temp_unfiltered, T.inverse().matrix());
		RegisterObs(temp_unfiltered);
	}
	
	return temp;
}


void MapDrawer::MergeUpdate()
{
	
    if(!octobusy || lockocto2)
    {
		cout<<"日 Initialize octomap merging"<<endl;
	    octo_mergebusy = true;
        cout<<"日 Start merging octomap..."<<endl;      
		PointCloud::Ptr cloud(new PointCloud());

        for (int i=0;i<currentvpKFs.size();i++)
        {
            for (int j=0;j<pointcloud1.size();j++)
            {
                if(pointcloud1[j].pcID==currentvpKFs[i]->mnFrameId)
                {
					PointCloud npcloud;
					
					Eigen::Isometry3d pose = ORB_SLAM3::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
                    PointCloud::Ptr _cloud(new PointCloud());

					pcl::copyPointCloud(*pointcloud1[j].pcE, npcloud); 

				    gndfilter(npcloud);

					octomap::point3d sensorOrigin = octomap::point3d( pose(0,3), pose(1,3), pose(2,3));// point cloud origin

					InsertScan(sensorOrigin, true_ground, true_nonground);// Insert new point cloud into octomap map====

					PlotOctomap();
					
				    std::cout<< "Merge optomap pieces: "<< j  << std::endl;
                    continue;
                }
            }
        }
        cout<<"日 Finalizing global octomap..."<<endl;	
			
        octo_mergebusy = false;
		lockocto2 = false;
        octo_mergecount++;

		cout<<"日 Octomap merging finished & update global map successfully!"<<endl;
		cout<<"日 Continuing SLAM process..."<<endl;	
    }
}


void MapDrawer::LoopUpdate()
{
    if(!octobusy || lockocto)
    {
		cout<<"日 Initialize octomap loop closing"<<endl;
	    octo_loopbusy = true;
        cout<<"日 Start loop closing octomap point..."<<endl; 
		PointCloud::Ptr cloud(new PointCloud());
		currentvpKFs = mpAtlas->GetAllKeyFrames();//  Get updated all keyframes=====  
		
        for (int i=0;i<currentvpKFs.size();i++)
        {
            for (int j=0;j<pointcloud1.size();j++)
            {
                if(pointcloud1[j].pcID==currentvpKFs[i]->mnFrameId)
                {
					PointCloud npcloud;
					
					Eigen::Isometry3d pose = ORB_SLAM3::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
                    PointCloud::Ptr _cloud(new PointCloud());

					pcl::copyPointCloud(*pointcloud1[j].pcE, npcloud); 

				    gndfilter(npcloud);

					octomap::point3d sensorOrigin = octomap::point3d( pose(0,3), pose(1,3), pose(2,3));// point cloud origin

					InsertScan(sensorOrigin, true_ground, true_nonground);// Insert new point cloud into octomap map====

					PlotOctomap();

					std::cout<< "Plot loop closing optomap pieces: "<< j  << std::endl;
                    continue;
                }
            }
        }
        cout<<"日 Finalizing global octomap..."<<endl;
					
        octo_loopbusy = false;
		lockocto = false;
        octo_loopcount++;

		cout<<"日 Octomap Loop closing finished & update global map successfully!"<<endl;
		cout<<"日 Continuing SLAM process..."<<endl;	
    }
}


void MapDrawer::gndfilter(PointCloud &temp)	//Ground Filter 1
{
	PointCloud ground;
	PointCloud nonground;
	true_ground = ground;
    true_nonground = nonground;


	if (filter_ground == 0) //Do not Filter ground from pcl
	{
		if(temp.size()<50)
		{
		    std::cout<<"日 Pointcloud too small skip ground plane extraction" << std::endl;
		    true_ground = temp;
		}
		else 
		{
			true_nonground = temp;
		}
	}
	else if(filter_ground == 1) //Filter out the ground from pcl
	{ 
		if(temp.size()<50)
		{
		    std::cout<<"日 Pointcloud too small skip ground plane extraction" << std::endl;
		    true_ground = temp;
		}
	// Here you can simply eliminate the points where the y-axis direction > the height of the robot and accelerate the removal of planes.
		else // Random sampling consistency model segmentation
		{
		    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);// model coefficient
		    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);// point cloud index

		    pcl::SACSegmentation<pcl::PointCloud<pcl::PointXYZRGB>::PointType> seg;//Split
		    seg.setOptimizeCoefficients(true);// Optimization coefficient
		    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);//split plane
		    seg.setMethodType(pcl::SAC_RANSAC);// Random sampling consistency split
		    seg.setMaxIterations(200);// Iterate 200 times
		    seg.setDistanceThreshold(0.04);// distance threshold
		    seg.setAxis(Eigen::Vector3f(0, 1 ,0));// xz Plane point cloud y-direction axis in the plane
		    seg.setEpsAngle(0.5);

		    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered(temp);// Point cloud before segmentation
		    pcl::ExtractIndices<pcl::PointCloud<pcl::PointXYZRGB>::PointType> extract;
		    bool groundPlaneFound = false; // ground plane found sign
		    while(cloud_filtered.size()>10 && !groundPlaneFound)// Ground plane not found
		    {
		        seg.setInputCloud(cloud_filtered.makeShared());// Segmenter input point cloud
		        seg.segment(*inliers, *coefficients);// Split
		        if(inliers->indices.size()==0)
		        {
		            break;
		        }
		        extract.setInputCloud(cloud_filtered.makeShared());// point cloud extractor
		        extract.setIndices(inliers);


				// a*X + b*Y + c*Z + d = 0;
		        if (std::abs(coefficients->values.at(3)) >0.07)// compare coefficient 
		        {
					std::cout << ">< Ground plane found..." << std::endl;
		            extract.setNegative (false);
		            extract.filter (true_ground); // Extract points on the plane and ground points
		            // remove ground points from full pointcloud:
		            // workaround for PCL:
		            if(inliers->indices.size() != cloud_filtered.size())
		            {
		              extract.setNegative(true);
		              pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
		              extract.filter(cloud_out);
		              true_nonground += cloud_out; // Point cloud without ground
		              cloud_filtered = cloud_out;
		            }

		            groundPlaneFound = true;
		        }
		        else
		        {
				std::cout << ">< Horizontal plane (not ground) found..." << std::endl;

		            pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
		            extract.setNegative (false);
		            extract.filter(cloud_out);
		            true_nonground +=cloud_out; // Plane not found

		            if(inliers->indices.size() != cloud_filtered.size())
		            {
		                 extract.setNegative(true);
		                 cloud_out.points.clear();
		                 extract.filter(cloud_out);
		                 cloud_filtered = cloud_out;
		            }
		            else
		            {
		                 cloud_filtered.points.clear();

		            }
		          }

		     }//while

		     if(!groundPlaneFound)
		     {
		         true_nonground = temp;
		     }
		}
	}

	else
	{
		std::cout << "Error: Undefined FilterGndData value! Use 0 or 1. Exiting.." << std::endl; 
		exit(0); 
	}
}



// Insert new point cloud into octomap map
void MapDrawer::InsertScan(octomap::point3d sensorOrigin,  // point cloud origin
                           pcl::PointCloud<pcl::PointXYZRGB> &ground, //ground
                           pcl::PointCloud<pcl::PointXYZRGB> &nonground)// non-ground
{

// Convert coordinates to key
    if(!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)||
        !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
     {
            printf(">< Could not generate key for origin\n");
     }

     octomap::KeySet free_cells, occupied_cells;// Free grid, occupied grid

// For each ground point cloud
     for(auto p:ground.points)
     {
        octomap::point3d point(p.x, p.y, p.z);
        // only clear space (ground points)
        if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
        {
             free_cells.insert(m_keyRay.begin(), m_keyRay.end()); // The ground is an empty grid
             m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);//color
        }
        octomap::OcTreeKey endKey;
        if(m_octree->coordToKeyChecked(point, endKey))
        {
              updateMinKey(endKey, m_updateBBXMin);
              updateMaxKey(endKey, m_updateBBXMax);
         }
        else
        {
              printf(">< Could not generator key for endpoint");
        }
     }

// Non-ground point cloud
// all other points : free on ray, occupied on endpoints:
     for(auto p:nonground.points)
     {
         octomap::point3d point(p.x, p.y, p.z);
         //free cell
         if(m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
         {
            // free_cells.insert(m_keyRay.begin(),m_keyRay.end()); // Non-idle
         }
         //occupided endpoint
         octomap::OcTreeKey key;
         if(m_octree->coordToKeyChecked(point, key))
         {
             occupied_cells.insert(key); // occupied grid
             updateMinKey(key, m_updateBBXMin);
             updateMaxKey(key, m_updateBBXMax);
             m_octree->averageNodeColor(p.x, p.y, p.z, p.r,p.g, p.b);
         }

     }

// free grid
     for(octomap::KeySet::iterator it = free_cells.begin(), 
                                   end= free_cells.end(); 
                                   it!=end; ++it)
     {
         if(occupied_cells.find(*it) == occupied_cells.end())// Occupied grid not found
         {
             m_octree->updateNode(*it, false);// free grid
         }
     }
// occupied grid
     for(octomap::KeySet::iterator it = occupied_cells.begin(), 
                                   end= occupied_cells.end(); 
                                   it!=end; ++it)
     {
         m_octree->updateNode(*it, true);// occupied grid
     }

     m_octree->prune();
}


void MapDrawer::RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs)
{
    observation = mvObs;
}

// Pangolin displays pcl point cloud points
void MapDrawer::DrawObs(void)
{
    glPointSize(mPointSize*2);// point size
    glBegin(GL_POINTS);// Start adding points
    //glColor3f(1.0,0.0,0.0);// color
    for(unsigned int i=0; i< observation.points.size(); i++)// Traverse each point
    {
       glColor3f(observation.points[i].r/255.0, 
                 observation.points[i].g/255.0,
                 observation.points[i].b/255.0);
        glVertex3f(observation.points[i].x,observation.points[i].y,observation.points[i].z);//Add points
    }
    glEnd();// End adding point
}

// Save map as octomap
void MapDrawer::SaveOctoMap(const std::string& filename)
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open())
    {
        m_octree->prune();
        m_octree->write(outfile);
        outfile.close();
    }
}

// if in DynamicRegion,return true, otherwise return fasle.
bool MapDrawer::isInDynamicRegion(int x,int y,std::vector<cv::Rect_<float> >& vDynamicBorder_)
{
    for(unsigned int i = 0; i < vDynamicBorder_.size(); i++)
    {
		cv::Rect_<float> rect1d = vDynamicBorder_[i];
		cv::Rect_<int> rect2d (rect1d.tl(), rect1d.br());
		rect2d = rect2d + cv::Point(-50,-50) + cv::Size(50,50); 
        if(x > int(rect2d.x) && x < int(rect2d.x + rect2d.width) && y > int(rect2d.y) && y < int(rect2d.y + rect2d.height))
            return true;
    }
    return false;
}


} //namespace ORB_SLAM3
