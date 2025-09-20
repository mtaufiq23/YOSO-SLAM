/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/


#ifndef YOLO_DETECT_H
#define YOLO_DETECT_H



#include <torch/torch.h>
#include <torch/script.h>
#include <torch/csrc/jit/passes/tensorexpr_fuser.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <iostream>
#include <utility>
#include <time.h>

#include <cassert>
#include <SimpleIni.h>
#include "Fps.h"
#include <mutex>


// Detected object information
typedef struct Object
{
    cv::Rect_<float> rect;  //bounding box
    float prob;             // confidence
    std::string object_name;// Object category name
    int class_id;           // Category id
} Object;


#include "KeyFrame.h"


namespace ORB_SLAM3
{

class Tracking;
class KeyFrame;


class YoloDetection
{
public:
    
	std::chrono::steady_clock::time_point start_time, end_time;
	std::chrono::milliseconds duration_ms;
	
	const char* DLmodel;
	size_t start;
	size_t end;
	std::string MODEL;
	std::string imgh_str;
	std::string imgw_str;
	std::string batsize_str;

	int imgh;
    int imgw;
	int detcol;
	int detrow;
    int batsize;

	const char* DLmodelpath;	
	std::string DLmodelpath_str;
	const char* _RUN_MODE;
	long RUN_MODE;
	const char* _COND_Thres;
	const char* _IOU_Thres;
	float COND_Thres;
	float IOU_Thres;
	const char* _NMS_ver;
	const char* _DISPLAY_MODE;
	long NMS_ver;
    long DISPLAY_MODE;
	const char* detobj;
	const char* detobjpath;
	const char* PREDEFINE_DYNAMIC_OBJ;
	std::string _PREDEFINE_DYNAMIC_OBJ;
	std::vector<std::string> splitWord(std::string& str);

    YoloDetection();
    ~YoloDetection();
    void GetImage(cv::Mat& RGB);
    void ClearImage();
    bool Detect();
    void ClearArea();
    std::vector<cv::Rect2i> mvPersonArea = {};

	torch::Tensor CreateTensorfEZ(const cv::Mat &_img, int batchsize, int channel, int imgH, int imgW);	
    std::vector<torch::Tensor> Prediction(torch::Tensor data, std::string& model_name, float _confThres, float _iouThres);
    std::vector<torch::Tensor> non_max_suppressionv2(torch::Tensor preds, float score_thresh, float iou_thresh);
	torch::Tensor xywh2xyxy(torch::Tensor x);
	torch::Tensor nms(torch::Tensor bboxes, torch::Tensor scores, float thresh);
    
    cv::Mat mRGB;
    torch::jit::script::Module mModule;
    std::vector<std::string> mClassnames;

    std::vector<std::string> mvDynamicNames;
    std::vector<cv::Rect_<float>> mvDynamicArea;
    std::map<std::string, std::vector<cv::Rect_<float>>> mmDetectMap;

    std::vector<Object> Objects; // 2d detection results static
	std::vector<Object> DyObjects; // 2d detection results dynamic
	std::vector<KeyFrame*> mvKeyframes;  // keyframe pointer array
	void InsertKeyFrame(KeyFrame* kf);
	std::queue<uint16_t> mvKFno;
    uint16_t kfno = 0;

    void ImageDetectFinished();

	std::mutex mMutexGetNewImage;
    std::mutex mMutexImageDetectFinished;
    bool mbNewImageFlag;
	Tracking* mpTracker;

    bool mbHaveDynamicObjectForMapping = false;
    bool mbHaveDynamicObjectForRmDynamicFeature = false;
    std::vector<cv::Rect_<float> > mvPotentialDynamicBorderForMapping;
    std::vector<cv::Rect_<float> > mvPotentialDynamicBorderForRmDynamicFeature;

};

} //namespace ORB_SLAM3

#endif //YOLO_DETECT_H
