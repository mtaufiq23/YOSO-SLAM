/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/


#ifndef FPS_H
#define FPS_H

extern int readings;
extern float avg_time [10], sum_time, avg_fps [10], sum_fps;
extern int time_reads;
extern int meas_fps;
extern int inf_per_img;


extern int p_readings;
extern float avg_ptime [10], psum_time, avg_pfps [10], sum_pfps;
extern int ptime_reads;
extern int pmeas_fps;
extern int pinf_per_img;

extern std::string model_display;

#endif //FPS_H

