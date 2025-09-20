/*
* This file is part of YOSO-SLAM.
* Copyright (C) 2025 Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya
* Universiti Teknologi Malaysia
* Co-authors: Hazlina Selamat and Anas Aburaya
*/

#include <YoloDetect.h>
#include "Tracking.h"


#define STRCAT(A, B) A B


enum class byte : std::uint8_t {};
int readings = 10;
float avg_time [10], sum_time = 0.0, avg_fps [10], sum_fps = 0.0;
int time_reads = 0;
int meas_fps = 0;
int inf_per_img = 0;

int p_readings = 10;
float avg_ptime [10], psum_time = 0.0, avg_pfps [10], sum_pfps = 0.0;
int ptime_reads = 0;
int pmeas_fps = 0;
int pinf_per_img = 0;

std::string model_display;


namespace ORB_SLAM3
{

YoloDetection::YoloDetection()
{
		
	CSimpleIniA ini;
	ini.SetUnicode();

	//Load Config.ini file
	SI_Error rc = ini.LoadFile("/home/mtaufiq23/Desktop/SLAM/YOSO-SLAM/Yolo_data/Config.ini");
	if (rc < 0) { /* handle error */ };

	
	DLmodel = ini.GetValue("Torchscript_File", "Module", "yolov8n.torchscript_320_320_b1");

    std::string DLmodel_str = DLmodel;

    // Find the string "torchscript"
    start = DLmodel_str.rfind("torchscript");

	// Find the char '.'
    end = DLmodel_str.find_last_of('.');
    
    // Extract model name
    MODEL = DLmodel_str.substr(0, end - 1);
	model_display = DLmodel_str.substr(0, end);;

    // Extract image height
    imgh_str = DLmodel_str.substr(start + 12, 3);

	// Extract image width
    imgw_str = DLmodel_str.substr(start + 16, 3);

	// Extract batch size
    batsize_str = DLmodel_str.substr(start + 21, 1);
 
    imgh = stoi(imgh_str);
    imgw = stoi(imgw_str);
	detcol = imgw;	
	detrow = imgh;
    batsize = stoi(batsize_str);

	DLmodelpath = ini.GetValue("Torchscript_File", "Module_Path", "/home/mtaufiq23/Desktop/SLAM/YOSO-SLAM/Yolo_data/");
	DLmodelpath_str = DLmodelpath;

	//Extract Confidence Score
	_COND_Thres = ini.GetValue("YOLO_Preset", "Confidence_Score", "0.4");

	//Extract IOU threshold
	_IOU_Thres = ini.GetValue("YOLO_Preset", "IOU_Threshold", "0.5");

	COND_Thres = atof(_COND_Thres);
	IOU_Thres = atof(_IOU_Thres);

	//Extract Display Mode
	_DISPLAY_MODE = ini.GetValue("YOLO_Preset", "Display_Mode", "2");

    DISPLAY_MODE = atoi(_DISPLAY_MODE);

	detobj = ini.GetValue("YOLO_Preset", "File_COCO_Object_List", "coco.names");

	detobjpath = ini.GetValue("YOLO_Preset", "COCO_path", "/home/mtaufiq23/Desktop/SLAM/YOSO-SLAM/Yolo_data/");

	PREDEFINE_DYNAMIC_OBJ = ini.GetValue("YOLO_Preset", "Predefine_Dynamic_Object", "Refer to config.ini file");

    
//    omp_set_num_threads(1);
//    torch::set_num_threads(4);

	//Load torchscript
    torch::jit::setTensorExprFuserEnabled(false);
    mModule = torch::jit::load (DLmodelpath_str + DLmodel_str);

    std::cout << "## Loaded Torchsript: <" << DLmodel_str << ">"  << std::endl;
    std::cout << "## Inference image resize: [ " << imgw << " x " << imgh << " ]" << std::endl;
  
    if (DISPLAY_MODE == 2) std::cout << "## Display mode: Detected dynamic objects only." << std::endl;
    else if (DISPLAY_MODE == 1) std::cout << "## Display mode: All detected objects." << std::endl;
    else std::cout << "Unsupported display mode!." << std::endl;
	
    std::cout << "## Current CPU core used: " << omp_get_max_threads() << std::endl;
    
	std::ifstream f((string)detobjpath + (string)detobj);
    std::string name = "";
    while (std::getline(f, name))
    {
        mClassnames.push_back(name);
    } 

	_PREDEFINE_DYNAMIC_OBJ = PREDEFINE_DYNAMIC_OBJ;	
	mvDynamicNames = {YoloDetection::splitWord(_PREDEFINE_DYNAMIC_OBJ)};
	
}


YoloDetection::~YoloDetection()
{

}


void YoloDetection::InsertKeyFrame(KeyFrame* kf)
{
    mvKFno.push(kfno);
	kfno++;
    mvKeyframes.push_back(kf);
}


bool YoloDetection::Detect()
{
	start_time = std::chrono::steady_clock::now();

	DyObjects.clear();
	Objects.clear();
                 
    cv::Mat img;
    if (!mRGB.isContinuous()) mRGB = mRGB.clone();	//Copying raw data from such a non-continuous cv::Mat would
													//cause the resulting Tensor to be messed up. Use cv::Mat::clone to make sure it is continuous:
    if(mRGB.empty())
    {
        std::cout << "Read RGB failed!" << std::endl;
        return false;
    }

    // Pre-processing ~ Preparing input tensor
    cv::resize(mRGB, img, cv::Size(imgw, imgh));
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

	torch::Tensor imgTensor;
	
    imgTensor = YoloDetection::CreateTensorfEZ(img, batsize, img.channels(), img.cols, img.rows);
    
	//Inferencing (Output have all boxes):
    std::vector<torch::Tensor> dets = YoloDetection::Prediction(imgTensor, MODEL, COND_Thres, IOU_Thres);

	mbHaveDynamicObjectForMapping = false;
    mbHaveDynamicObjectForRmDynamicFeature = false;
    mvPotentialDynamicBorderForRmDynamicFeature.clear();
    mvPotentialDynamicBorderForMapping.clear();

   //Post-processing/detection output with optimized boxes
    if (dets.size() > 0)
    {
        // Visualize result
        for (size_t i=0; i < dets[0].sizes()[0]; ++ i)
        {
            float left = dets[0][i][0].item().toFloat() * mRGB.cols / detcol;//640;
            float top = dets[0][i][1].item().toFloat() * mRGB.rows / detrow;//384;
            float right = dets[0][i][2].item().toFloat() * mRGB.cols / detcol;//640;
            float bottom = dets[0][i][3].item().toFloat() * mRGB.rows / detrow;//384;
            float confidence = dets[0][i][4].item().toFloat();
            int classID = dets[0][i][5].item().toInt();

			//Temporary Store dynamic object for display in cmd
			if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
			{
				Object dyobject;
				dyobject.class_id = classID;
				dyobject.object_name = std::string(mClassnames[classID]);
				dyobject.prob = confidence;
				dyobject.rect.x = left;
				dyobject.rect.y = top;
				dyobject.rect.width = right - left;
				dyobject.rect.height = bottom - top;

		    	DyObjects.emplace_back(dyobject);   
                
                mbHaveDynamicObjectForMapping = true;
                mvPotentialDynamicBorderForMapping.emplace_back(dyobject.rect);
       
                mbHaveDynamicObjectForRmDynamicFeature = true;
                mvPotentialDynamicBorderForRmDynamicFeature.emplace_back(dyobject.rect);                
			}
		    else //Store static object which later add to obj database
			{
				Object object;
				object.class_id = classID;
				object.object_name = std::string(mClassnames[classID]);
				object.prob = confidence;
				object.rect.x = left;
				object.rect.y = top;
				object.rect.width = right - left;
				object.rect.height = bottom - top;

		    	Objects.push_back(object);
	        }
         
			if (DISPLAY_MODE == 1) //Display all
			{
		        cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));
	            mmDetectMap[mClassnames[classID]].push_back(DetectArea);	//Collects all detected objects & sent to viewer to display

		        if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
		        {
		            cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
		            mvDynamicArea.push_back(DynamicArea);
		        }
	            else mmDetectMap = mmDetectMap;
			}

			else if (DISPLAY_MODE == 2) //Display dynamic obj only
			{
				cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));

		        if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
		        {
		            cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
		            mvDynamicArea.push_back(DynamicArea);
		            mmDetectMap[mClassnames[classID]].push_back(DynamicArea); //Collect only detected specified dynamic objects & sent to viewer to display
		        }
			}

			else if (DISPLAY_MODE == 3)	//Display none
			{
		        cv::Rect2i DetectArea(left, top, (right - left), (bottom - top));
		        if (count(mvDynamicNames.begin(), mvDynamicNames.end(), mClassnames[classID]))
		        {
		            cv::Rect2i DynamicArea(left, top, (right - left), (bottom - top));
		            mvDynamicArea.push_back(DynamicArea);
		        }
	            else mmDetectMap = mmDetectMap;
			}

			else 
			{
            	std::cout << "Error: Undefined DISPLAY_MODE! Exiting.." << std::endl; 
		        exit(0); 
			}
        }

        if (mvDynamicArea.size() == 0)
        {
            cv::Rect2i tDynamicArea(1, 1, 1, 1);
            mvDynamicArea.push_back(tDynamicArea);
        }
    }

	end_time = std::chrono::steady_clock::now();

    duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	start_time = {};
	end_time = {}; 
    
	if (time_reads < readings) 
	{
		avg_time[time_reads] = duration_ms.count();
        time_reads++;
	}
	else 
	{
		for(int i = 0; i < readings; ++i)
		{
		    sum_time += avg_time[i];
		}
		inf_per_img = static_cast<int>(sum_time/readings);
		time_reads = 0;
		sum_time = 0;
	}

    return true; 

}


vector<torch::Tensor> YoloDetection::Prediction(torch::Tensor data, string& model_name, float _confThres, float _iouThres)
{
	auto pred = mModule.forward({ data });
	
	if (model_name == "yolov8")//yolov8 
	{
		torch::Tensor pT = pred.toTensor();
		torch::Tensor score = std::get<0>(pT.slice(1, 4, -1).max(1, true));
		data = torch::cat({pT.slice(1, 0, 4), score, pT.slice(1, 4, -1)}, 1).permute({0, 2, 1});
	}	
    else
	{
        std::cout << "Error: Unsupported torch model! Exiting.." << std::endl; 
		exit(0);
	}
		
    return non_max_suppressionv2(data, _confThres, _iouThres); 
}


vector<torch::Tensor> YoloDetection::non_max_suppressionv2(torch::Tensor prediction, float confThres, float iouThres)
{
	torch::Tensor xc = prediction.select(2, 4) > confThres;
	int maxWh = 4096;
	int maxNms = 30000;
	std::vector<torch::Tensor> output;
	for (int i = 0; i < prediction.size(0); i++)
	{
		output.push_back(torch::zeros({ 0, 6 }));
	}
	for (int i = 0; i < prediction.size(0); i++)
	{
		torch::Tensor x = prediction[i];
		x = x.index_select(0, torch::nonzero(xc[i]).select(1, 0));
		if (x.size(0) == 0) continue;
		
		x.slice(1, 5, x.size(1)).mul_(x.slice(1, 4, 5));
		torch::Tensor box = xywh2xyxy(x.slice(1, 0, 4));
		std::tuple<torch::Tensor, torch::Tensor> max_tuple = torch::max(x.slice(1, 5, x.size(1)), 1, true);
		x = torch::cat({ box, std::get<0>(max_tuple), std::get<1>(max_tuple) }, 1);
		x = x.index_select(0, torch::nonzero(std::get<0>(max_tuple) > confThres).select(1, 0));
		int n = x.size(0);
		if (n == 0)
		{
			continue;
		}
		else if (n > maxNms)
		{
			x = x.index_select(0, x.select(1, 4).argsort(0, true).slice(0, 0, maxNms));
		}
		torch::Tensor c = x.slice(1, 5, 6) * maxWh;
		torch::Tensor boxes = x.slice(1, 0, 4) + c, scores = x.select(1, 4);
		torch::Tensor ix = nms(boxes, scores, iouThres).to(x.device());
		output[i] = x.index_select(0, ix).cpu();
	}
	return output;
}


torch::Tensor YoloDetection::xywh2xyxy(torch::Tensor x)
{
	torch::Tensor y = x.clone();
	y.select(1, 0) = x.select(1, 0) - x.select(1, 2) / 2;
	y.select(1, 1) = x.select(1, 1) - x.select(1, 3) / 2;
	y.select(1, 2) = x.select(1, 0) + x.select(1, 2) / 2;
	y.select(1, 3) = x.select(1, 1) + x.select(1, 3) / 2;
	return y;
}


torch::Tensor YoloDetection::nms(torch::Tensor bboxes, torch::Tensor scores, float thresh)
{
	auto x1 = bboxes.select(1, 0);
	auto y1 = bboxes.select(1, 1);
	auto x2 = bboxes.select(1, 2);
	auto y2 = bboxes.select(1, 3);
	auto areas = (x2 - x1) * (y2 - y1);
	auto tuple_sorted = scores.sort(0, true);
	auto order = std::get<1>(tuple_sorted);

	std::vector<int> keep;
	while (order.numel() > 0) 
	{
		if (order.numel() == 1) 
		{
			auto i = order.item();
			keep.push_back(i.toInt());
			break;
		}
		else 
		{
			auto i = order[0].item();
			keep.push_back(i.toInt());
		}

		auto order_mask = order.narrow(0, 1, order.size(-1) - 1);

		auto xx1 = x1.index({ order_mask }).clamp(x1[keep.back()].item().toFloat(), 1e10);
		auto yy1 = y1.index({ order_mask }).clamp(y1[keep.back()].item().toFloat(), 1e10);
		auto xx2 = x2.index({ order_mask }).clamp(0, x2[keep.back()].item().toFloat());
		auto yy2 = y2.index({ order_mask }).clamp(0, y2[keep.back()].item().toFloat());
		auto inter = (xx2 - xx1).clamp(0, 1e10) * (yy2 - yy1).clamp(0, 1e10);

		auto iou = inter / (areas[keep.back()] + areas.index({ order.narrow(0,1,order.size(-1) - 1) }) - inter);
		auto idx = (iou <= thresh).nonzero().squeeze();
		if (idx.numel() == 0) 
		{
			break;
		}
		order = order.index({ idx + 1 });
	}
	return torch::tensor(keep);
}


void YoloDetection::GetImage(cv::Mat &RGB)
{
    mRGB = RGB;
}


void YoloDetection::ClearImage()
{
    mRGB = 0;
}


void YoloDetection::ClearArea()
{
    mvPersonArea.clear();
}


void YoloDetection::ImageDetectFinished()
{
	std::unique_lock <std::mutex> lock(mMutexImageDetectFinished);
    mpTracker->mbDetectImageFinishedFlag=true;
}


torch::Tensor YoloDetection::CreateTensorfEZ(const cv::Mat &_img, int batchsize, int channel, int imgH, int imgW)
{
//### Tensor input using torch empty/zeros ###
	torch::Tensor _imgTensor = torch::empty({batchsize, imgW, imgH, channel}, torch::kByte);	//#[faster] Create pytorch.pt tensor input (1,640,384,3)(-->b,w,h,c) 
    std::memcpy(_imgTensor.data_ptr(),_img.data,sizeof(byte)*_imgTensor.numel());	//Copy all image data to tensor
																														
    _imgTensor = _imgTensor.permute({0,3,1,2});		//# Transpose to match yolov8 input shape (1,3,640,640)-->(b,c,w,h)
    _imgTensor = _imgTensor.toType(torch::kFloat);	//# Convert type to float
    _imgTensor = _imgTensor.div(255);				//# Convert RGB pixel values which are usually in the range [0, 255] to [0.0, 1.0] range

	return {_imgTensor};
}


// Function to split words from the given string.
vector<string> YoloDetection::splitWord(string& str)
{
    // Find length of given variable
    int n = str.length();
 
    // Create an empty string
    string word = "";
    string q = "";
	vector<string> p;
 
    // Iterate over the string character by character using
    // For loop
    for (int i = 0; i < n; i++) {
 
        // Check if the current iteration is equal to ' ' or
        // it's the last character
        if (str[i] == ' ' or i == (n - 1)) {
            q += word;
			p.push_back(q);
            word = "";
            q = "";
        }
 
        // Add current character in word string
        else {
            word += str[i];
        }
    }
	return p;
}

} //namespace ORB_SLAM3
