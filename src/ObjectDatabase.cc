/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

/* 
 Semantic Database 
 Add, delete, and merge detected object data
 */

#include "ObjectDatabase.h"


// Searching by name 
bool Cluster::operator ==(const std::string &x){
    return(this->object_name == x);
} 


ObjectDatabase::ObjectDatabase(const std::string &strSettingPath)
{
    DataBaseSize = 0;
    mClusters.clear();
//  Set cuboid color. Different colors correspond to different objects 
    for (int i = 0; i < 80; i++) // with background
    {  // coco data set 80 types of objects
       mvColors.push_back(cv::Scalar( i*2+10, i*2+10, i*2+10));
       // person
       // ...
       // ...
       // toothbrush
    }
    mvColors[39] = cv::Scalar(255,0,127); // bottle pink  
    mvColors[56] = cv::Scalar(255,0,0);   // chair red
//    mvColors[0] = cv::Scalar(0,0,255);  // person blue
    mvColors[62] = cv::Scalar(255,0,255);  // monitor violet
//	mvColors[57] = cv::Scalar(255,255,0);  // sofa violet
	mvColors[64] = cv::Scalar(255,255,0);  // mouse yellow
    mvColors[66] = cv::Scalar(127,0,255);  // keyb seablue
	mvColors[65] = cv::Scalar(255,128,0);  // remote orange
    mvColors[77] = cv::Scalar(0,0,0);  // teddy black
    
// Use this to Set Default Object size or set manually at Tolerance Distance below. Adjust them in .yaml file.
    for (int i = 0; i < 80; i++)  
    { // coco data set 80 types of objects
      mvSizes.push_back(0.6);
    }

//Tolerance Distance that make it appear or considered as same object in the scene
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	mvSizes[0] = fSettings["Semantic.person"];
	mvSizes[39] = fSettings["Semantic.bottle"];
	mvSizes[56] = fSettings["Semantic.chair"];	
	mvSizes[58] = fSettings["Semantic.pottedplant"];
	mvSizes[60] = fSettings["Semantic.diningtable"];
	mvSizes[62] = fSettings["Semantic.tvmonitor"];	
	mvSizes[63] = fSettings["Semantic.laptop"];
	mvSizes[64] = fSettings["Semantic.mouse"];
	mvSizes[66] = fSettings["Semantic.keyboard"];
	mvSizes[73] = fSettings["Semantic.book"];
	mvSizes[77] = fSettings["Semantic.teddy_bear"];
}


// class destructor
ObjectDatabase::~ObjectDatabase()
{
}
// Return the defined object color
cv::Scalar  ObjectDatabase::getObjectColor(int class_id)
{
   return mvColors[class_id];
}
// Return the defined object size
float ObjectDatabase::getObjectSize(int class_id)
{
   return mvSizes[class_id];
}       
// Return object data with the same name in the database
std::vector<Cluster>  ObjectDatabase::getObjectByName(std::string objectName)
{

        // Check whether the object is in the database by name
	std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
	std::vector<Cluster>::iterator it_end = mClusters.end(); 
        std::vector<Cluster> sameName;// Objects with the same name 
	while(true) 
        {
	    iter = find(++iter, it_end, objectName);// Search by name
	    if (iter != it_end )// Find one and store it
                sameName.push_back(*iter);
	    else//cannot find it anymore
	        break;  
	}
        return sameName; 
}

//Adding Object into Database
void ObjectDatabase::addObject(Cluster& cluster)
{
    // A. Check the total quantity, if the database is empty, join directly
    if(!mClusters.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mClusters.push_back(cluster);
        return;
    }
    else
    {

    // B. The object already exists in the database. Find whether the new object already exists in the database.
	std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
	std::vector<Cluster>::iterator it_end = mClusters.end(); 
        std::vector<std::vector<Cluster>::iterator> likely_obj;// an iterator for the object of the same name
	while(true) 
    {
	    iter = find(++iter, it_end, cluster.object_name);// Search by name
	    if (iter != it_end )// Find one and store it away
                likely_obj.push_back(iter);
	    else//Can't find it anymore
	        break;  
	}

    // C. If not found, add it directly to the database
        std::vector<Cluster>::iterator best_close;// recent index
        float center_distance=100;// corresponding distance
        if(!likely_obj.size())
        {
            DataBaseSize++;
            cluster.object_id = DataBaseSize;
            mClusters.push_back(cluster);
            return;
        }
        else//Find multiple objects with the same name in the database
        {
            
    // D. Go through every object with the same name and find the one with the closest center point
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;// Center point connection vector
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // The shortest distance
                    best_close      = temp_iter;// The corresponding index
                }
            }

    // E. If the distance is smaller than the size of the object, it is considered to be the same object in the same space, 
    //    and the information about the object in the database is updated.
			if (center_distance < mvSizes[cluster.class_id])
            {
                best_close->prob    = (best_close->prob + cluster.prob)/2.0; // Comprehensive confidence
                best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // Center average
                best_close->size     = (best_close->size + cluster.size)/2.0; // Center size
                best_close->rot_mat = cluster.rot_mat;
            }
            else 
            {
    // F. If the distance exceeds the size of the object, it is considered to be the same object at 
    //    different locations and placed directly into the database.
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mClusters.push_back(cluster); 
            }
			
        }
    }

// Database size limit. Targets with low confidence will be deleted if exceeds the size limit.
    return; 
}



