/**
* Description: This file is part of parkingEnvSensing.
* Date: 2017-06-20
* Final Edit: 2017-06-20
*/

#include "SlamDataPub.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <random>

#include <opencv2/core/eigen.hpp>

#include <mutex>

namespace ORB_SLAM3
{

SlamDataPub::SlamDataPub(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Atlas* pAtlas):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking), mpAtlas(pAtlas),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
	
    // camera under ground      
    mInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;  // camera coordinate represented in ground coordinate system
    
	//mInitCam2Ground_t.setZero();    
	mInitCam2Ground_t << 0, 0, 0;
	
    //mTrans_cam2ground.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	mTrans_cam2ground << 1, 0, 0, 0,
						 0, 1, 0, 0,
						 0, 0, 1, 0,
						 0, 0, 0, 1;

	
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;  //< block_rows, block_cols >(pos_row, pos_col)

    // camera under vehicle, 
    mCam2Vehicle_R << 0,0,1,-1,0,0,0,-1,0;  // camera coordinate represented in vehicle coordinate system
    //mCam2Vehicle_t.setZero();
	mCam2Vehicle_t << 0, 0, 0;
    //mTrans_cam2vehicle.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	mTrans_cam2vehicle << 1, 0, 0, 0,
						  0, 1, 0, 0,
						  0, 0, 1, 0,
						  0, 0, 0, 1;
	
    mTrans_cam2vehicle.block<3,3>(0,0) = mCam2Vehicle_R;
    mTrans_cam2vehicle.block<3,1>(0,3) = mCam2Vehicle_t;  //< block_rows, block_cols >(pos_row, pos_col)
	cout << "strSettingPath=" << strSettingPath << endl;
	mbGetNewCamPose = false;
	mbGetPointCloud = false;
	mbGetDrawFrame = false;
    //mbGetPointCloudMapping = false;

    mpPointCloudMapping = new PointCloudMapping();

	//cv::namedWindow("Current Frame", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Current Frame", int(200 * mImageWidth / mImageHeight), 200);

}

PointCloudMapping* SlamDataPub::getPointCloudMapping()
{
    return mpPointCloudMapping;
}


void SlamDataPub::TrackingDataPub()
{
    geometry_msgs::PoseStamped camPose2Ground;  
    geometry_msgs::PoseStamped vehiclePose2Ground;  
    nav_msgs::Path cameraPath, vehiclePath;
    while(1)
    { 
	    if(mbGetNewCamPose)
	    {
            GetCurrentROSCameraMatrix(camPose2Ground);
            GetCurrentROSVehicleMatrix(vehiclePose2Ground);
            GetCurrentROSTrajectories(cameraPath, vehiclePath);
            CamPose_pub_.publish(camPose2Ground);  
            VehiclePose_pub_.publish(vehiclePose2Ground);
            CamPath_pub_.publish(cameraPath);   // KeyFrames
            VehiclePath_pub_.publish(vehiclePath);

            float tf_q_x = vehiclePose2Ground.pose.orientation.x;
            float tf_q_y = vehiclePose2Ground.pose.orientation.y;
            float tf_q_z = vehiclePose2Ground.pose.orientation.z;
            float tf_q_w = vehiclePose2Ground.pose.orientation.w;
            float tf_x = vehiclePose2Ground.pose.position.x;
            float tf_y = vehiclePose2Ground.pose.position.y;
            float tf_z = vehiclePose2Ground.pose.position.z;

            Vehicle2Ground_broadcaster_.sendTransform(
            tf::StampedTransform(
            tf::Transform(tf::Quaternion(tf_q_x,tf_q_y,tf_q_z,tf_q_w), tf::Vector3(tf_x, tf_y, tf_z)),
            ros::Time::now(),"ground", "vehicle"));  

            mbGetNewCamPose = false; 
	    
	    }
        if(CheckFinish())
            break;  
        usleep(1*1000); 
    }
   
}

void SlamDataPub::PointCloudPub()
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    while(1)
    {
        if(mbGetPointCloud)
        {
            GetCurrentROSAllPointCloud(allMapPoints, referenceMapPoints);
            AllPointCloud_pub_.publish(allMapPoints);
            RefPointCloud_pub_.publish(referenceMapPoints);
            mbGetPointCloud = false;
        }
	    if(CheckFinish())
            break;  
        usleep(mT*1000/2); 
    }
    
}

void SlamDataPub::CloudPointMappingPub()
{
    sensor_msgs::PointCloud2 allMapPoints;
    while(1)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloud( new pcl::PointCloud<pcl::PointXYZRGB> );  
        mpPointCloudMapping->generatePointCloud(globalCloud, mpAtlas, mTrans_cam2ground);
        //cout << "globalCloud->points.size()=" << globalCloud->points.size() << endl;
        if (globalCloud->points.size() > 0)
        {
            pcl::PCLPointCloud2 pcl_pc1;
            pcl::toPCLPointCloud2(*globalCloud, pcl_pc1);    // pcl::PointXYZRGB -> pcl::PCLPointCloud2
            pcl_conversions::fromPCL(pcl_pc1, allMapPoints);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
            allMapPoints.header.frame_id = "ground";  
            allMapPoints.header.stamp = ros::Time::now();  

            //cout << "globalCloud->points.size()=" << globalCloud->points.size() << endl;
            CloudPoint_pub_.publish(allMapPoints);
        }

        if(CheckFinish())
            break;  
        usleep(mT*1000 / 2); 
        //cout << "CloudPointMappingPub end" << endl;
    }
}

void SlamDataPub::DrawFramePub()
{
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.header.stamp = ros::Time::now();
    while(1)
    {
        if(mbGetDrawFrame)
        {  
            cv::Mat img = mpFrameDrawer->DrawFrame();
            //cv::imshow("Current Frame",img);
            //cv::waitKey(1); //mT/2);
            cvi.image = img;
            sensor_msgs::Image im;
            cvi.toImageMsg(im);
            DrawFrame_pub_.publish(im);
            mbGetDrawFrame = false;
        }
        if(CheckFinish())
            break;  
        usleep(1*1000); 
    }
 
}

void SlamDataPub::Run()
{
    mbFinished = false;
    mbStopped = false;

    CamPose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("camera_pose",1);
    VehiclePose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("vehicle_pose",1);
    CamPath_pub_ = nh.advertise<nav_msgs::Path>("camera_path",1);
    VehiclePath_pub_ = nh.advertise<nav_msgs::Path>("vehicle_path",1);
    AllPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_all",1);
    RefPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_ref",1);
    CloudPoint_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloudPointMapping",1);
    
    image_transport::ImageTransport it_(nh);
    DrawFrame_pub_ = it_.advertise("/frame_now", 1);
       
    thread threadCamPosePub(&SlamDataPub::TrackingDataPub,this);   
    thread threadDrawFramePub(&SlamDataPub::DrawFramePub,this); 
    thread threadPointCloudPub(&SlamDataPub::PointCloudPub,this); 
    thread threadCloudPointMappingPub(&SlamDataPub::CloudPointMappingPub, this);
 
    threadPointCloudPub.join();
    threadCamPosePub.join(); 
    
    SetFinish();
}

void SlamDataPub::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SlamDataPub::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SlamDataPub::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SlamDataPub::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void SlamDataPub::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool SlamDataPub::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool SlamDataPub::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void SlamDataPub::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void SlamDataPub::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbGetNewCamPose = true;
	mbGetPointCloud = true;
	mbGetDrawFrame = true;
}

void SlamDataPub::GetCurrentROSCameraMatrix(geometry_msgs::PoseStamped &cam_pose)
{
      if(!mCameraPose.empty())
      {
		Eigen::Matrix4f cam_pose2firstcam;
		Eigen::Matrix4f cam_pose2ground;
		{
			unique_lock<mutex> lock(mMutexCamera);
			cv2eigen(mCameraPose.inv(),cam_pose2firstcam);
			cam_pose2ground = mTrans_cam2ground * cam_pose2firstcam;
			mCam2GroundNow_T.block<4,4>(0,0) = cam_pose2ground;
			
		}
		cam_pose.pose.position.x = cam_pose2ground(0,3);
		cam_pose.pose.position.y  = cam_pose2ground(1,3);
		cam_pose.pose.position.z  = cam_pose2ground(2,3);
		
		Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
		Eigen::Quaternionf q(Rwc);
		cam_pose.pose.orientation.x = q.x();
		cam_pose.pose.orientation.y = q.y();
		cam_pose.pose.orientation.z = q.z();
		cam_pose.pose.orientation.w = q.w();
		
		cam_pose.header.frame_id = "ground";
		cam_pose.header.stamp = ros::Time::now(); 
      }
      
}

void SlamDataPub::GetCurrentROSVehicleMatrix(geometry_msgs::PoseStamped &vehicle_pose)
{
      if(!mCameraPose.empty())
      {
		Eigen::Matrix4f vehicle_pose2ground;
	
		vehicle_pose2ground = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
	
		mVehicle2GroundNow_T.block<4,4>(0,0) = vehicle_pose2ground;
	
		vehicle_pose.pose.position.x = vehicle_pose2ground(0,3);
		vehicle_pose.pose.position.y  = vehicle_pose2ground(1,3);
		vehicle_pose.pose.position.z  = vehicle_pose2ground(2,3);
		
		Eigen::Matrix3f Rwc = vehicle_pose2ground.block<3,3>(0,0);
		Eigen::Quaternionf q(Rwc);
		vehicle_pose.pose.orientation.x = q.x();
		vehicle_pose.pose.orientation.y = q.y();
		vehicle_pose.pose.orientation.z = q.z();
		vehicle_pose.pose.orientation.w = q.w();

		vehicle_pose.header.frame_id = "ground";
		vehicle_pose.header.stamp = ros::Time::now();  	
      }
}

void SlamDataPub::GetCurrentROSTrajectories(nav_msgs::Path &cam_path, nav_msgs::Path &vehicle_path)
{
    if(!mCameraPose.empty())
    {  
        nav_msgs::Path cam_path_temp;
        nav_msgs::Path vehicle_path_temp;

        geometry_msgs::PoseStamped cam_pose;
        geometry_msgs::PoseStamped vehicle_pose;

        vector<cv::Mat> currentTrajectory;	
        mpSystem->GetCurrentTrajectory(currentTrajectory);

        Eigen::Matrix4f cam_pose_temp;

        for(auto mt:currentTrajectory) // no need to inverse
        {
            cv2eigen(mt,cam_pose_temp);
            
            Eigen::Matrix4f cam_pose2ground = mTrans_cam2ground * cam_pose_temp;
            Eigen::Matrix4f vehicle_pose2ground = cam_pose2ground * mTrans_cam2vehicle.inverse();
            
            cam_pose.pose.position.x = cam_pose2ground(0,3);
            cam_pose.pose.position.y = cam_pose2ground(1,3);
            cam_pose.pose.position.z = cam_pose2ground(2,3);
            Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
            Eigen::Quaternionf q(Rwc);	      
            cam_pose.pose.orientation.x = q.x();
            cam_pose.pose.orientation.y = q.y();
            cam_pose.pose.orientation.z = q.z();
            cam_pose.pose.orientation.w = q.w();
            
            vehicle_pose.pose.position.x = vehicle_pose2ground(0,3);
            vehicle_pose.pose.position.y = vehicle_pose2ground(1,3);
            vehicle_pose.pose.position.z = vehicle_pose2ground(2,3);
            Eigen::Matrix3f Rwc2 = vehicle_pose2ground.block<3,3>(0,0);
            Eigen::Quaternionf q2(Rwc2);	      
            vehicle_pose.pose.orientation.x = q2.x();
            vehicle_pose.pose.orientation.y = q2.y();
            vehicle_pose.pose.orientation.z = q2.z();
            vehicle_pose.pose.orientation.w = q2.w();
            
            vehicle_path_temp.poses.push_back(vehicle_pose);
            cam_path_temp.poses.push_back(cam_pose);	     
        }
        cam_path_temp.header.frame_id = "ground";
        cam_path_temp.header.stamp = ros::Time::now();   
        vehicle_path_temp.header.frame_id = "ground";
        vehicle_path_temp.header.stamp = ros::Time::now(); 

        cam_path = cam_path_temp;
        vehicle_path = vehicle_path_temp;
    }
}

void SlamDataPub::GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGB> );  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGB> );     
    
    const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();
    
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;
	
    default_random_engine e;
    uniform_int_distribution<unsigned> u(0, 255);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        
        //cv::Mat Xw = (cv::Mat_<float>(3,1) << pMP->GetWorldPos()(0), pMP->GetWorldPos()(1), pMP->GetWorldPos()(2));
        //cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGB p1;
        Eigen::Vector4f p1_temp, p1_temp_t;
        //p1_temp(0) = pos.at<float>(0);
        //p1_temp(1) = pos.at<float>(1);
        //p1_temp(2) = pos.at<float>(2);

        p1_temp(0) = vpMPs[i]->GetWorldPos()(0);
        p1_temp(1) = vpMPs[i]->GetWorldPos()(1);
        p1_temp(2) = vpMPs[i]->GetWorldPos()(2);

        p1_temp(3) = 1; 
        p1_temp_t = mTrans_cam2ground * p1_temp;	
        p1.x = p1_temp_t(0);
        p1.y = p1_temp_t(1);
        p1.z = p1_temp_t(2);
        p1.b = 255;
        p1.g = 255;
        p1.r = 255;

        //p1.b = u(e);
        //p1.g = u(e);
        //p1.r = u(e);

        p1.a = 255;
        cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGB -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "ground";  
    all_point_cloud.header.stamp = ros::Time::now();   

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        //cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGB p2;
        Eigen::Vector4f p2_temp, p2_temp_t;
        //p2_temp(0) = pos.at<float>(0);
        //p2_temp(1) = pos.at<float>(1);
        //p2_temp(2) = pos.at<float>(2);
        p2_temp(0) = (*sit)->GetWorldPos()(0);
        p2_temp(1) = (*sit)->GetWorldPos()(1);
        p2_temp(2) = (*sit)->GetWorldPos()(2);

        p2_temp(3) = 1;
        p2_temp_t = mTrans_cam2ground * p2_temp;	
        p2.x = p2_temp_t(0);
        p2.y = p2_temp_t(1);
        p2.z = p2_temp_t(2);

        p2.b = 0;
        p2.g = 0;
        p2.r = 255;

        //p2.b = u(e);
        //p2.g = u(e);
        //p2.r = u(e);

        p2.a = 255;
        cloud_ref->points.push_back( p2 );
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "ground";
    ref_point_cloud.header.stamp = ros::Time::now();   

}


Eigen::Matrix4f SlamDataPub::getTransCam2Ground()
{
    return mTrans_cam2ground;
}

}


