/*
* This file is a modified version of ORB-SLAM3.<https://github.com/UZ-SLAMLab/ORB_SLAM3>
*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/


#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <opencv2/highgui.hpp>

#include <mutex>


namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	    Octomap_Status = fSettings["OctoMap.Run"];
        octosavepath = fSettings["OctoMap.FileSavePath"].string();
		octomansavepath = fSettings["OctoMap.ManualFileSavePath"].string();
		octobaksavepath = fSettings["OctoMap.BackupFileSavePath"].string();
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

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;
//	mT = 1000/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();

}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;
//	mT = 1000/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

	Octomap_Status = fSettings["octoMap.Run"];
	octosavepath = fSettings["octoMap.FileSavePath"].string();
	octomansavepath = fSettings["octoMap.ManualFileSavePath"].string();
	octobaksavepath = fSettings["octoMap.BackupFileSavePath"].string();

    return !b_miss_params;
}

void Viewer::Run()
{

	if(mpTracker->mSensor == mpSystem->RGBD)
	{
    	mbFinished = false;
		mbStopped = false;

		auto &win = pangolin::CreateWindowAndBind("YOSO-SLAM: Map Viewer",1024,768);
		win.Move(1920,0); // Top left

		// 3D Mouse handler requires depth testing to be enabled
		glEnable(GL_DEPTH_TEST);

		// Issue specific OpenGl we might need
		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(185));
		pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
		pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
		pangolin::Var<bool> menuTopView("menu.Top View",false,false);
		// pangolin::Var<bool> menuSideView("menu.Side View",false,false);

		pangolin::Var<bool> menuShowPoints("menu.Show Points",false,true);
		pangolin::Var<bool> menuShowCurrentFrames("menu.Show CurrentFrames",true,true);
		pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
		pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
		pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
		pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
		pangolin::Var<bool> menuReset("menu.Reset",false,false);

		pangolin::Var<bool> menuSave("menu.Save Octomap", false, false);
		pangolin::Var<bool> menuSaveAtlas("menu.Save Atlas", false, false);

		pangolin::Var<bool> menuStop("menu.Stop",false,false);
		pangolin::Var<bool> menuStepByStep("menu.Pause",false,true);  // false, true	//Change to RGBD slam pause/stop button
//		pangolin::Var<bool> menuStep("menu.Step",false,false);

		pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
		// Define Camera Render Object (for view / scene browsing)
		pangolin::OpenGlRenderState s_cam(
		            pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
		            pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
		            );

		// Add named OpenGL viewport to window and provide 3D Handler
		pangolin::View& d_cam = pangolin::CreateDisplay()
		        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
		        .SetHandler(new pangolin::Handler3D(s_cam));

		pangolin::OpenGlMatrix Twc, Twr;
		Twc.SetIdentity();
		pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
		Ow.SetIdentity();

	    cv::namedWindow("YOSO-SLAM: Current Frame");
		cv::moveWindow("YOSO-SLAM: Current Frame", 0, 0);
		
		bool bFollow = true;
		bool bLocalizationMode = false;
		bool bStepByStep = false;
		bool bCameraView = true;

		float trackedImageScale = mpTracker->GetImageScale();

		if (Octomap_Status == 0 ) menuShowPoints = true;


		cout << "Starting the Viewer" << endl;
		while(1)
		{
		    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

		    if(mbStopTrack)
		    {
//		        menuStepByStep = true;
		        mbStopTrack = false;
		    }

		    if(menuFollowCamera && bFollow)
		    {
		        if(bCameraView)
		            s_cam.Follow(Twc);
		        else
		            s_cam.Follow(Ow);
		    }
		    else if(menuFollowCamera && !bFollow)
		    {
		        if(bCameraView)
		        {
		            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
		            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
		            s_cam.Follow(Twc);
		        }
		        else
		        {
		            s_cam.Follow(Ow);
		        }
		        bFollow = true;
		    }
		    else if(!menuFollowCamera && bFollow)
		    {
		        bFollow = false;
		    }

		    if(menuCamView)
		    {
		        menuCamView = false;
		        bCameraView = true;
		        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
		        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
		        s_cam.Follow(Twc);
		    }

		    if((menuTopView && mpMapDrawer->mpAtlas->isImuInitialized()) || (menuTopView && mpMapDrawer->mpAtlas))
		    {
		        menuTopView = false;
		        bCameraView = false;
		        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
		        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(20,-20,-8, 0,0,0, 0.0,0.0,1.0));
		        s_cam.Follow(Ow);
		    }

		    if(menuLocalizationMode && !bLocalizationMode)
		    {
		        mpSystem->ActivateLocalizationMode();
		        bLocalizationMode = true;
				mpMapDrawer->bIsLocalization = true;
				
		    }
		    else if(!menuLocalizationMode && bLocalizationMode)
		    {
		        mpSystem->DeactivateLocalizationMode();
		        bLocalizationMode = false;
				mpMapDrawer->bIsLocalization = false;
		    }

		    if(menuStepByStep && !bStepByStep)	//Became pause option for rgbd slam
		    {
		        cout << "SLAM RGBD pause..." << endl;
		        mpTracker->SetStepByStep(true);
		        bStepByStep = true;
		    }
		    else if(!menuStepByStep && bStepByStep)
		    {
				cout << "SLAM RGBD continue..." << endl;
		        mpTracker->SetStepByStep(false);
		        bStepByStep = false;
		    }

		    d_cam.Activate(s_cam);
		    glClearColor(1.0f,1.0f,1.0f,1.0f);

			// Draw grid in xz plane
		    mpMapDrawer->DrawGrid();

			if(menuShowCurrentFrames)
		    	mpMapDrawer->DrawCurrentCamera(Twc);

		    if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
		        mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba);

			if (Octomap_Status==1 && mpMapDrawer->mpAtlas->isCreatedMap) 
			{
				//Clear current octomap & started a new one
				if (mpMapDrawer->ot_exist == false && mpMapDrawer->vkfHasVal == true && Octomap_Status==1) mpMapDrawer->ClearOctomap();	
				mpSystem->mpAtlas->isCreatedMap = false;
			} 

		    if(menuShowPoints)
		        mpMapDrawer->DrawMapPoints();
			else
		    { 		// Octomap display
					if (Octomap_Status == 1 )
					{
						//Plot octomap from save file if it exist
						if(mpMapDrawer->ot_exist == true) 
						{
							mpMapDrawer->PlotOctomap();
							cout << "Offline octomap builded successfully." << endl;
						}

						mpMapDrawer->DrawOctoMap();// show octomap 
			            
						if(mpMapDrawer->octodrawobj == 1)
						{
							glLineWidth(5);    // Line width
					        mpMapDrawer->DrawObject();// Draw target object 3D border
						}
						
        				if(mpTracker->bStepByStep) 
						{
							pangolin::FinishFrame();
							continue;
						}
					}
		            else if (Octomap_Status == 0){} 
					else
					{
						std::cout << "Error: Undefined octoMap.Run value! Use 0 or 1. Exiting.." << std::endl; 
						exit(0); 
					}
		    }

		    pangolin::FinishFrame();

		    cv::Mat toShow;
		    cv::Mat im = mpFrameDrawer->DrawFrame(mpTracker, trackedImageScale);

		    if(both){
		        cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
		        cv::hconcat(im,imRight,toShow);
		    }
		    else{
		        toShow = im;
		    }

		    if(mImageViewerScale != 1.f)
		    {
		        int width = toShow.cols * mImageViewerScale;
		        int height = toShow.rows * mImageViewerScale;
		        cv::resize(toShow, toShow, cv::Size(width, height));
		    }

		    // Yolo bounding box drawing 
			if (mpSystem->Yolo_Status==1){
		    {
		        std::unique_lock<std::mutex> lock(mMutexPAFinsh);
		        for (auto vit = mmDetectMap.begin(); vit != mmDetectMap.end(); vit++)
		        {
		            if (vit->second.size() != 0)
		            {
		                for (auto area : vit->second)
		                {
		                    cv::rectangle(toShow, area, cv::Scalar(51, 153, 255), 1);
		                    cv::putText(toShow,
		                                vit->first,
		                                cv::Point(area.x, area.y),
		                                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(153, 0, 0), 2);
		                }
		            }

		        }
		    }}
		    // Yolo bounding box drawing

			cv::moveWindow("YOSO-SLAM: Current Frame", 0, 0);
		    cv::imshow("YOSO-SLAM: Current Frame",toShow);
			
			
		    cv::waitKey(mT);

		    if(menuReset)
		    {
				menuShowGraph = false;
				menuShowInertialGraph = true;
				menuShowCurrentFrames = true;
				menuShowKeyFrames = true;
				menuShowPoints = false;
				menuLocalizationMode = false;
			    if(bLocalizationMode)
				mpSystem->DeactivateLocalizationMode();
			    bLocalizationMode = false;
				mpMapDrawer->bIsLocalization = false;
				bFollow = true;
				menuFollowCamera = true;
				mpSystem->ResetActiveMap();

				mpSystem->mpAtlas->isCreatedMap = true;
		        menuReset = false;
		    }			

		    if(menuStop)
		    {
		        if(bLocalizationMode)
		            mpSystem->DeactivateLocalizationMode();

		        // Stop all threads
		        mpSystem->Shutdown();

		        // Save camera trajectory
		        mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
		        mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
		        menuStop = false;
		    }

		    if(Stop())
		    {				
		        while(isStopped())
		        {
		            usleep(3000);
		        }
		    }

			 if(menuSave && Octomap_Status==1) // octomap
		    {
			   if(mpTracker->mSensor == mpSystem->RGBD)
			   {
					mpMapDrawer->SaveOctoMap(octomansavepath);// Save manual octomap map
					cout<<"Save Octomap(RGBD) done!"<<endl;
		       }
			
			   else cout<<"*Saving failed! Octomap only for RGBD inputs. Use 'Save Atlas' instead."<<endl;

			   menuSave = false;
		    }

			if(menuSaveAtlas) // Save sparse map (atlas) 
			{
			   if(mpTracker->mSensor == mpSystem->RGBD)
			   {
			   	  mpSystem->SaveAtlas2(FileType::BINARY_FILE, "atlas/mAtlas-rgbd");
			   }			   			   
			   menuSaveAtlas = false;
		       cout<<"Save Atlas done!"<<endl;
			}

		    if(CheckFinish())
		        break;
		}

		SetFinish();
	}
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}


} //namespace ORB_SLAM3
