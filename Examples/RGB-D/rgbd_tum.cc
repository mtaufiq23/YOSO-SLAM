/*
* This file is a modified version of ORB-SLAM3.<https://github.com/UZ-SLAMLab/ORB_SLAM3>
*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

#include "Fps.h"

using namespace std;



void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

		double ttrack= std::chrono::duration_cast<std::chrono::milliseconds >(t2 - t1).count();


       //Measure actual fps & slam process per image
	    std::chrono::milliseconds img_ms = std::chrono::duration_cast<std::chrono::milliseconds> (t2 - t1);		

		if (ptime_reads < p_readings) 
		{
//			avg_ptime[ptime_reads] = img_ms.count();
			avg_pfps[ptime_reads] = 1000/img_ms.count();
		    ptime_reads++;
		}
		else 
		{
			for(int i = 0; i < p_readings; ++i)
			{
//				psum_time += avg_ptime[i];
				sum_pfps += avg_pfps[i];
			}
//			pinf_per_img = static_cast<int>(psum_time/p_readings);
			int _pmeas_fps = static_cast<int>(sum_pfps/p_readings);
	        if (_pmeas_fps > 30) pmeas_fps = 30;
            else pmeas_fps = _pmeas_fps;
			ptime_reads = 0;
//			psum_time = 0;
		    sum_pfps = 0;
		}



        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

//		if (SLAM.mbShutDown) break;
    }

		
	                           
	//Octomap
	if (SLAM.mpViewer->Octomap_Status==1)
	{
		//Loop octomap
		while(SLAM.mpMapDrawer->octo_loopbusy || SLAM.mpMapDrawer->octobusy)
		{
		    cout<<"";
		}
		SLAM.mpMapDrawer->octo_bStop = true;

		//Merge pcl
		while(SLAM.mpMapDrawer->octo_mergebusy || SLAM.mpMapDrawer->octobusy)
		{
		    cout<<"";
		}
		SLAM.mpMapDrawer->octo_bStop2 = true;

		//Save octomap
		//	SLAM.OctoMapSave("/home/mtaufiq23/Desktop/SLAM/Analysis/octomap.ot");// Save octomap map
		SLAM.OctoMapSave(SLAM.mpViewer->octosavepath);// Save octomap map
	}

	
         
         
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "Shutdown successfully " << endl;
    cout << "---------------------------------------------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory  
    SLAM.SaveTrajectoryTUM("f_dataset_RGBD.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("kf_dataset_RGBD.txt"); 

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}



