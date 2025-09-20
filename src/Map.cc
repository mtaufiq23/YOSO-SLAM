/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Map.h"

#include<mutex>
#include <sys/stat.h>// File status===


namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

//void Map::PreSave(std::set<GeometricCamera*> &spCams)
//{
//    int nMPWithoutObs = 0;
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        if(!pMPi || pMPi->isBad())
//            continue;

//        if(pMPi->GetObservations().size() == 0)
//        {
//            nMPWithoutObs++;
//        }
//        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
//        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
//        {
//            if(it->first->GetMap() != this || it->first->isBad())
//            {
//                pMPi->EraseObservation(it->first);
//            }

//        }
//    }

//    // Saves the id of KF origins
//    mvBackupKeyFrameOriginsId.clear();
//    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
//    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
//    {
//        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
//    }


//    // Backup of MapPoints
//    mvpBackupMapPoints.clear();
//    for(MapPoint* pMPi : mspMapPoints)
//    {
//        if(!pMPi || pMPi->isBad())
//            continue;

//        mvpBackupMapPoints.push_back(pMPi);
//        pMPi->PreSave(mspKeyFrames,mspMapPoints);
//    }

//    // Backup of KeyFrames
//    mvpBackupKeyFrames.clear();
//    for(KeyFrame* pKFi : mspKeyFrames)
//    {
//        if(!pKFi || pKFi->isBad())
//            continue;

//        mvpBackupKeyFrames.push_back(pKFi);
//        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
//    }

//    mnBackupKFinitialID = -1;
//    if(mpKFinitial)
//    {
//        mnBackupKFinitialID = mpKFinitial->mnId;
//    }

//    mnBackupKFlowerID = -1;
//    if(mpKFlowerID)
//    {
//        mnBackupKFlowerID = mpKFlowerID->mnId;
//    }

//}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;

    std::set<MapPoint*> tmp_mspMapPoints1;
    tmp_mspMapPoints1.insert(mspMapPoints.begin(), mspMapPoints.end());

    for(MapPoint* pMPi : tmp_mspMapPoints1)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }

        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();

    std::set<MapPoint*> tmp_mspMapPoints2;
    tmp_mspMapPoints2.insert(mspMapPoints.begin(), mspMapPoints.end());

    for(MapPoint* pMPi : tmp_mspMapPoints2)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }


    if(mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if(mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}


// Write map points
void Map::_WriteMapPoint(ofstream &f, MapPoint* mp) 
{
  f.write((char*)&mp->mnId, sizeof(mp->mnId));               // id: long unsigned int
  cv::Mat wp = mp->_GetWorldPos();// Location x,y,z 
  f.write((char*)&wp.at<float>(0), sizeof(float));           // pos x: float
  f.write((char*)&wp.at<float>(1), sizeof(float));           // pos y: float
  f.write((char*)&wp.at<float>(2), sizeof(float));           // pos z: float
}

// Write keyframes
void Map::_WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp) 
{
  f.write((char*)&kf->mnId, sizeof(kf->mnId));                 // Keyframe id: long unsigned int
  f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));     // Timestamp ts: TimeStamp, double

#if 0
  cerr << "writting keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
  cerr << " parent " << kf->GetParent() << endl;
  cerr << "children: ";
  for(auto ch: kf->GetChilds())
	cerr << " " << ch->mnId;
  cerr <<endl;
  cerr << kf->mnId << " connected: (" << kf->GetConnectedKeyFrames().size() << ") ";
  for (auto ckf: kf->GetConnectedKeyFrames())
	cerr << ckf->mnId << "," << kf->GetWeight(ckf) << " ";
  cerr << endl;
#endif
  


  cv::Mat Tcw = kf->_GetPose();// Key frame pose ===
  f.write((char*)&Tcw.at<float>(0,3), sizeof(float));          // px: float
  f.write((char*)&Tcw.at<float>(1,3), sizeof(float));          // py: float
  f.write((char*)&Tcw.at<float>(2,3), sizeof(float));          // pz: float
  vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
  f.write((char*)&Qcw[0], sizeof(float));                      // qx: float
  f.write((char*)&Qcw[1], sizeof(float));                      // qy: float
  f.write((char*)&Qcw[2], sizeof(float));                      // qz: float
  f.write((char*)&Qcw[3], sizeof(float));                      // qw: float

  f.write((char*)&kf->N, sizeof(kf->N));                       // Number of feature points nb_features: int
  for (int i=0; i<kf->N; i++) 
  {
        // Feature points==============
	cv::KeyPoint kp = kf->mvKeys[i];
	f.write((char*)&kp.pt.x,     sizeof(kp.pt.x));               // float
	f.write((char*)&kp.pt.y,     sizeof(kp.pt.y));               // float
	f.write((char*)&kp.size,     sizeof(kp.size));               // float
	f.write((char*)&kp.angle,    sizeof(kp.angle));              // float
	f.write((char*)&kp.response, sizeof(kp.response));           // float
	f.write((char*)&kp.octave,   sizeof(kp.octave));             // int
        
	// descriptor
	for (int j=0; j<32; j++) 
	  f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

	unsigned long int mpidx; 
        MapPoint* mp = kf->GetMapPoint(i); // map point
	if (mp == NULL) mpidx = ULONG_MAX;
	else mpidx = idx_of_mp[mp]; // All map points located in map id
	f.write((char*)&mpidx,   sizeof(mpidx));  // long int
  }

}



// Save binary map===================
bool Map::Save(const string &filename) 
{
  cerr << "Map: Saving to " << filename << endl;
  ofstream f;
  f.open(filename.c_str(), ios_base::out|ios::binary);
  
  cerr << "  writing " << mspMapPoints.size() << " mappoints" << endl;
  unsigned long int nbMapPoints = mspMapPoints.size();// Number of map points
  f.write((char*)&nbMapPoints, sizeof(nbMapPoints));  // Write the total number of map points

// Write each map point====
  for(auto mp: mspMapPoints)
	_WriteMapPoint(f, mp);

  map<MapPoint*, unsigned long int> idx_of_mp; // Map point:id mapping====
  unsigned long int i = 0;
  for(auto mp: mspMapPoints) 
  {
	idx_of_mp[mp] = i; // mapping===
	i += 1;
  }

// Write each keyframe =======  
  cerr << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
  unsigned long int nbKeyFrames = mspKeyFrames.size();
  f.write((char*)&nbKeyFrames, sizeof(nbKeyFrames)); // Write the number of keyframes
  for(auto kf: mspKeyFrames)
	_WriteKeyFrame(f, kf, idx_of_mp);// Write a keyframe=====

// Save the parent keyframe id, common view relationship and weight, minimum spanning tree======
  // store tree and graph
  for(auto kf: mspKeyFrames) {
	KeyFrame* parent = kf->GetParent();// Parent keyframe id
	unsigned long int parent_id = ULONG_MAX; 
	if (parent) parent_id = parent->mnId;
	f.write((char*)&parent_id, sizeof(parent_id));

	unsigned long int nb_con = kf->GetConnectedKeyFrames().size();// associated keyframes
	f.write((char*)&nb_con, sizeof(nb_con));
	for (auto ckf: kf->GetConnectedKeyFrames()) 
        {
	  int weight = kf->GetWeight(ckf); // Correlation weight (number of map points seen together)
	  f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
	  f.write((char*)&weight, sizeof(weight));
	}
  }

  f.close();
  cerr << "Map: finished saving" << endl;
  struct stat st;
  stat(filename.c_str(), &st);// Get file size
  cerr << "Map: saved " << st.st_size << " bytes" << endl;

#if 0
  for(auto mp: mspMapPoints)
	if (!(mp->mnId%100))
	  cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

  return true;


}

} //namespace ORB_SLAM3
