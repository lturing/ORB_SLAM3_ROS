#include "PointCloudMapping.h"
#include<vector>

namespace ORB_SLAM3 {

PointCloudData::PointCloudData(){
    cloud = std::make_shared<PointCloud>();
}

PointCloudData::~PointCloudData()
{
}

PointCloudMapping::PointCloudMapping()
{

    mbUpdateCloudPoint = false;
}

PointCloudMapping::~PointCloudMapping()
{
}


void PointCloudMapping::insertKeyFrame(KeyFrame* kf, const cv::Mat& color, const cv::Mat& depth, cv::Mat& mK, cv::Mat& mDistCoef)
{
    unique_lock<mutex> locker(mKeyFrameMtx);
    
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    int stride = 3;
    for (int i = 0; i < color.rows; i += stride)
    {
        for (int j = 0; j < color.cols; j+= stride)
        {
            cv::KeyPoint kp;
            kp.pt.x = i;
            kp.pt.y = j;
            mvKeys.push_back(kp);
        }
    }

    int N = mvKeys.size();

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);

    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
    mat=mat.reshape(1);

    int cnt = 0;
    PointCloud::Ptr cloud( new PointCloud() );
    float v_min = 1000000000.0;
    float v_max = 0.0;

    for ( int m=0; m<depth.rows; m+=stride )
    {
        for ( int n=0; n<depth.cols; n+=stride )
        {
            float d = depth.ptr<float>(m)[n];
            v_min = v_min > d ? d : v_min; 
            v_max = v_max > d ? v_max : isnan(d) ? v_max : d;

            if (isnan(d) || d < 0.01 || d>10)
            {
                cnt += 1;
                continue;
            }
            
            PointT p;
            p.x = ( mat.at<float>(cnt, 0) - kf->cx) * d / kf->fx;
            p.y = ( mat.at<float>(cnt, 1) - kf->cy) * d / kf->fy;
            p.z = d;
            /*
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
            */
            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back(p);
            cnt += 1;
        }
    }

    cout << "v_max=" << v_max << ", v_min=" << v_min << endl; 

    cloud->width = cloud->points.size();
    cloud->height = 1;

    PointCloudData pcd;
    pcd.kf = kf;
    pcd.cloud = cloud;

    mPointCloudDatas.push_back(pcd);
    mbUpdateCloudPoint = true;

}


void PointCloudMapping::generatePointCloud(PointCloud::Ptr& GlobalCloud, Atlas* pAtlas, Eigen::Matrix4f& Trans_cam2ground)
{ 

    if (!mbUpdateCloudPoint)
        return; 

    const vector<KeyFrame*> vpKFs = pAtlas->GetAllKeyFrames();
    std::set<long unsigned int> sKFs;
    for (auto kf : vpKFs)
    {
        sKFs.insert(kf->mnFrameId);
    }

    std::unique_lock<std::mutex> locker(mPointCloudMtx);   
    
    for (auto pcd : mPointCloudDatas)
    {
        if (sKFs.find(pcd.kf->mnFrameId) == sKFs.end())
            continue; 
        
        Eigen::Isometry3d T = Converter::toSE3Quat( pcd.kf->GetPoseInverse() );
        PointCloud::Ptr cloud(new PointCloud);
        pcl::transformPointCloud(*pcd.cloud, *cloud, T.matrix()); 
        
        T = Converter::toSE3Quat(Converter::toCvMat(Trans_cam2ground));

        PointCloud::Ptr cloud1(new PointCloud);
        pcl::transformPointCloud(*cloud, *cloud1, T.matrix()); 

        *GlobalCloud += *cloud1;
    }

    mbUpdateCloudPoint = false;

}


}