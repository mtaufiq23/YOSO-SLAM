# YOSO-SLAM: A Real-time Object Visual SLAM for Dynamic Scenes with Semantic 3-Dimensional Mapping

This work presents YOSO-SLAM (YOLO Semantic Object SLAM), an enhanced real-time VSLAM framework that generates a semantic three-dimensional (3D) Map for dynamic scenes while maintaining manageable computational demands and high pose estimation accuracy. YOSO-SLAM incorporates the ORB-SLAM3 pipeline and integrates YOLOv8 for object detection and dynamic feature removal.


## License
YOSO-SLAM is released under a GPLv3 License
if you use YOSO-SLAM in an academic work, please cite:
```bibtex
@misc{muslim2025yososlam,
  title        = {YOSO-SLAM: A Real-time Object Visual SLAM for Dynamic Scenes with Semantic 3-Dimensional Mapping},
  author       = {Mohd Taufiq Muslim, Hazlina Selamat and Anas Aburaya},
  year         = {2025},
<<<<<<< HEAD
  note         = {Manuscript under review. Code available at \url{https://github.com/mtaufiq23/YOSO-SLAM}}
=======
  note         = {Manuscript under review. Code available at \url{https://github.com/your-username/yoso-slam}}
>>>>>>> bff3f46 (Upload files)
}
```


## Build YOSO-SLAM on Host Machine
### 1. Prerequisistes
#### C++11 or C++14 Compiler
Default is c++14 but user can change to c++11 by editing the CMakeLists.txt

#### Pangolin
We use Pangolin for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

#### OpenCV
We use OpenCV to manipulate images and features. Download and install instructions can be found at: http://opencv.org. Tested with OpenCV 3.4.0, 4.5.0 and 4.9.0.

#### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. Tested with 3.1.0.

#### DBoW2, Sophus and g2o (Included in Thirdparty folder)

We use modified versions of the DBoW2 library to perform place recognition and g2o library to perform non-linear optimizations. Both modified libraries and all files included are BSD licensed. Sophus library for manipulating 2D and 3D rigid body transformations (MIT license).

#### Pcl
Required for point cloud generation and manipulation. Download and install instructions can be found at: https://github.com/PointCloudLibrary/pcl. Tested with pcl 1.1.2 and 1.1.3

#### Octomap
Required for generating 3D map based on octree. Download and install instructions https://github.com/OctoMap/octomap. Tested with 1.9.8.

<<<<<<< HEAD
#### libtorch (Included in Thirdparty folder)
=======
#### libtorch
>>>>>>> bff3f46 (Upload files)
Required for torchscript implementation in c++. Tested with 2.0.1+cpu.


### 2. Building YOSO-SLAM library and examples
Clone the repository
```
git clone https://github.com/mtaufiq23/YOSO-SLAM
```

Use the provided build.sh script to build the third-party libraries and YOSO-SLAM. For configuration details, refer to CMakeLists.txt.
```
cd YOSO-SLAM
chmod +x build.sh
./build.sh
```

### 3. TUM RGB-D Dataset Example
- Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.
- Associate RGB and depth images using the provided Python script: [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools):
```
python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
```
- Execute the following command, replacing the placeholders as described as follows: <Path To Settings>: Path to the appropriate settings file (TUM1.yaml, TUM2.yaml, or TUM3.yaml). <Path To Dataset>: Path to the uncompressed sequence folder. <Path To Association File>: Path to the corresponding associations file. An example can be found in the provided test.sh file.
```
/Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt <Path To Settings> <Path to Dataset> <Path To Association file>
```

## Acknowledgements
YOSO-SLAM is developed based on the ORB-SLAM3 framework [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Maintainer
This repository is maintained by [Mohd Taufiq Muslim](https://github.com/mtaufiq23)
