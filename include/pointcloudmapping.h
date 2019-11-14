/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

// STL
#include <condition_variable>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

// ORB_SLAM2
#include "System.h"
#include "PointCloude.h"

using namespace std;
using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    
    PointCloudMapping(const std::string &strSettingPath, bool bUseViewer=true);
    
    void save();
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs );
    
    void shutdown();
    
    void run();
    
    void updateCloseLoopCloud();
    
    int loopcount = 0;
    vector<KeyFrame*> currentvpKFs;
    
    // control flag
    bool mbCloudBusy;
    bool mbLoopBusy;
    bool mbStop;
    
protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    
    shared_ptr<thread>  mThdRunning;
    
    // shutdown
    mutex   shutDownMutex;
    bool    mbShutDownFlag;
    
    // store keyframe
    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;
    vector<PointCloude, Eigen::aligned_allocator<Eigen::Isometry3d> >     pointcloud;
    
    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    vector<cv::Mat>         colorImgks;
    vector<cv::Mat>         depthImgks;
    vector<int>             ids;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;
    
    // statistical filter
    pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
    double meank_ = 50;
    double thresh_ = 1;
    
    // voxel grid filter
    pcl::VoxelGrid<PointT>  voxel;
    double resolution_ = 0.04;

//! yonghui add
public:
    /**
     * @brief Set point cloud update flag
     *
     * @param bFlag
     */
    void setPointCloudMapUpdatedFlag(bool bFlag);
    
    /**
     * @brief Get point cloud update flag
     *
     * @return
     */
    bool getPointCloudMapUpdatedFlag();

    /**
     * @brief Thread-safe interface to get whole point cloud map
     *
     * @return
     */
    bool getGlobalCloud(PointCloud::Ptr &pCloud);
    
    /**
     * @brief Thread-safe interface to get obstacle part of cloud map
     * after performing plane segmentation.
     *
     * If prohibit plane segmentation, return false
     *
     * @param pCloud
     *
     * @return return false if plane segmentation is prohibited
     */
    bool getObstacleCloud(PointCloud::Ptr &pCloud);
    
    /**
     * @brief Thread-safe interface to get plane part of cloud map
     * after performing plane segmentation.
     *
     * If prohibit plane segmentation, return false
     *
     * @param pCloud
     *
     * @return return false if plane segmentation is prohibited
     */
    bool getPlaneCloud(PointCloud::Ptr &pCloud);
    
    /**
     * @brief Thread-safe interface to get plane coefficients
     * after performing plane segmentation.
     *
     * If prohibit plane segmentation, return false
     */
    bool getPlaneCoeffs(Eigen::Vector4d &planeCoeffs);
    
    /**
     * @brief Thread-safe interface to update footprint<--optical extrinsic matrix
     *
     * @param Rbc
     * @param tbc
     */
    void updateTbc(const Eigen::Matrix3d &Rbc, const Eigen::Vector3d &tbc);
    
    /**
     * @brief Thread-safe interface to update footprint<--optical extrinsic matrix
     */
    void updateTbc(const Eigen::Matrix4d &Tbc);
    
    /**
     * @brief Thread-safe interface to footprint<--optical extrinsic matrix
     *
     * @param Tbc
     */
    void getTbc(Eigen::Matrix4d &Tbc);
    
    void setUsePlaneSegmentationFlag(bool bFlag);
    
    bool getUsePlanSegmentationFlag();
    
    /**
     * @brief Calculate the transform matrix T from source plane to target plane:
     * target_plane = T^(-T)*source_plane
     *
     */
    void getPlaneTransformMatrix(const Eigen::Vector4d &target_plane, const Eigen::Vector4d &source_plane, Eigen::Matrix4d &T);
    
    /**
     * @brief Segment plane with RANSAC
     *
     * @param pPclMap
     * @param pPclPlane
     * @param planeCoeffs
     * @return
     */
    bool planeSACSegmentation(PointCloud::Ptr &pPclMap, PointCloud::Ptr &pPclPlane,
                              Eigen::Vector4d &planeCoeffs, const Eigen::Vector4d &lastPlaneCoeffs,
                              pcl::PointIndices::Ptr &pInliers);

protected:
    /**
     * @brief Perform plane segmentation for each point clouds of KeyFrame
     *
     * @param pPclFrame
     * @param pPclPlane
     * @return
     */
    bool framePlaneSegmentation(PointCloud::Ptr &pPclFrame, PointCloud::Ptr &pPclPlane);
    
    // plane segment flag and parameters
    mutex mMtxPlaneSegmentation;
    bool mbUsePlaneSegmentation;
    bool mbSegmentPerFrame;
    double mfPlaneDistThres;
    double mfFramePlaneDistThres;
    Eigen::Vector4d mPlaneCoeffs;
    Eigen::Vector4d mLastPlaneCoeffs;
    
    // output point cloud
    mutex mMtxTbcUpdated;
    Eigen::Matrix4d mTbc;
    double mfCameraHeight;
    
    // point cloud update
    PointCloud::Ptr mpPclObstacle;
    PointCloud::Ptr mpPclGroundPlane;
    
    // point cloud updated complete flag
    mutex mMtxPointCloudUpdated;
    PointCloud::Ptr mpPclGlobalMap;
    
    bool mbPointCloudMapUpdated;
    
    // pcl viewer
    bool mbUseViewer;
    // pcl::visualization::CloudViewer mViewer;
};

#endif // POINTCLOUDMAPPING_H
