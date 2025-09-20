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

#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

/*#include "System.h"*/

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// Object semantic information
typedef struct Cluster
{
 Eigen::Matrix3f rot_mat;  //rotation matrix
 Eigen::Vector3f size;    // 3d box size
 Eigen::Vector3f centroid;// Point cloud center point
 float prob;              // Confidence or probability
 std::string object_name; // Object class name
 int class_id;            // Corresponding category id
 int object_id;           // Object number
 bool operator ==(const std::string &x);
} Cluster;


class ObjectDatabase
{
public:
    ObjectDatabase(const std::string &strSettingPath);
    ~ObjectDatabase();
    void addObject(Cluster& cluster);
    cv::Scalar  getObjectColor(int class_id); // Defined object color
    float getObjectSize(int class_id);        // Defined object size

    std::vector<Cluster>  getObjectByName(std::string objectName);// Returns the object data with the same name in the database

    std::vector<Cluster> mClusters;   // Semantic point cloud target array
protected:
    std::vector<cv::Scalar> mvColors;// the color of each object
    std::vector<float>      mvSizes; // size of each object
    int DataBaseSize; 

	//Tolerance distance value that make it appear as same obj
	float person_toldist;
	float bicycle_toldist;
	float car_toldist;
	float motorbike_toldist;
	float aeroplane_toldist;
	float bus_toldist;
	float train_toldist;
	float truck_toldist;
	float boat_toldist;
	float traffic_light_toldist;
	float fire_hydrant_toldist;
	float stop_sign_toldist;
	float parking_meter_toldist;
	float bench_toldist;
	float bird_toldist;
	float cat_toldist;
	float dog_toldist;
	float horse_toldist;
	float sheep_toldist;
	float cow_toldist;
	float elephant_toldist;
	float bear_toldist;
	float zebra_toldist;
	float giraffe_toldist;
	float backpack_toldist;
	float umbrella_toldist;
	float handbag_toldist;
	float tie_toldist;
	float suitcase_toldist;
	float frisbee_toldist;
	float skis_toldist;
	float snowboard_toldist;
	float sports_ball_toldist;
	float kite_toldist;
	float baseball_bat_toldist;
	float baseball_glove_toldist;
	float skateboard_toldist;
	float surfboard_toldist;
	float tennis_racket_toldist;
	float bottle_toldist;
	float wine_glass_toldist;
	float cup_toldist;
	float fork_toldist;
	float knife_toldist;
	float spoon_toldist;
	float bowl_toldist;
	float banana_toldist;
	float apple_toldist;
	float sandwich_toldist;
	float orange_toldist;
	float broccoli_toldist;
	float carrot_toldist;
	float hot_dog_toldist;
	float pizza_toldist;
	float donut_toldist;
	float cake_toldist;
	float chair_toldist;
	float sofa_toldist;
	float pottedplant_toldist;
	float bed_toldist;
	float diningtable_toldist;
	float toilet_toldist;
	float tvmonitor_toldist;
	float laptop_toldist;
	float mouse_toldist;
	float remote_toldist;
	float keyboard_toldist;
	float cell_phone_toldist;
	float microwave_toldist;
	float oven_toldist;
	float toaster_toldist;
	float sink_toldist;
	float refrigerator_toldist;
	float book_toldist;
	float clock_toldist;
	float vase_toldist;
	float scissors_toldist;
	float teddy_bear_toldist;
	float hair_drier_toldist;
	float toothbrush_toldist;



};

#endif // OBJECTDATABASE_H
