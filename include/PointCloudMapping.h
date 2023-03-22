#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>  // Eigen核心部分
#include <Eigen/Geometry> // 提供了各种旋转和平移的表示

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>  


#include "KeyFrame.h"
#include "Converter.h"
#include "Atlas.h"

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H


typedef pcl::PointXYZRGB PointT; // A point structure representing Euclidean xyz coordinates, and the RGB color.
typedef pcl::PointCloud<PointT> PointCloud;

namespace ORB_SLAM3 {

class Converter;
class KeyFrame;
class Atlas;


class PointCloudData
{
public:
    PointCloudData();
    ~PointCloudData();
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
public:
    PointCloud::Ptr cloud;
public:
    KeyFrame* kf;
      
};

class PointCloudMapping {
    public:
        PointCloudMapping();
        ~PointCloudMapping();
        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth); // 传入的深度图像的深度值单位已经是m

        void insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth, cv::Mat& mK, cv::Mat& mDistCoef);
        void generatePointCloud(PointCloud::Ptr& GlobalCloud, Atlas* pAtlas, Eigen::Matrix4f& Trans_cam2ground);

        std::mutex mKeyFrameMtx;
        std::mutex mPointCloudMtx;
        std::vector<PointCloudData> mPointCloudDatas;
        bool mbUpdateCloudPoint;

};

}
#endif