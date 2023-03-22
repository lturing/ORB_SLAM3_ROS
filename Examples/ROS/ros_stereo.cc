/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <sstream>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;

std::chrono::steady_clock::time_point tt1;
std::chrono::steady_clock::time_point tt2;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};


class ImagePub
{
    // for kitti dataset
    public:
        ImagePub(const string& strSequence, image_transport::Publisher& leftImgPub, image_transport::Publisher& rightImgPub)
            :strSequence(strSequence), leftImgPub_(leftImgPub), rightImgPub_(rightImgPub) {}

        void LoadImages();
        void PubliserImages();
        
        string strSequence;
        image_transport::Publisher leftImgPub_, rightImgPub_;
        vector<string> vstrLeftImgFilenames, vstrRightImgFilenames;
        vector<double> vTimestamps;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo");
    ros::start();
    ros::NodeHandle nh;
    
    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }    
    
    string voc_dir = argv[2];
    string config_dir = argv[1];
    string strSeq = argv[3];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_dir,config_dir,ORB_SLAM3::System::STEREO,false);

    ImageGrabber igb(&SLAM);

    //stringstream ss(argv[3]);
    //ss >> boolalpha >> igb.do_rectify;
    igb.do_rectify = false;
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(config_dir, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));


    image_transport::ImageTransport iLeft_(nh);
    image_transport::Publisher imgPubLeft;
    imgPubLeft = iLeft_.advertise("/camera/left/image_raw", 5);

    image_transport::ImageTransport iRight_(nh);
    image_transport::Publisher imgPubRight;
    imgPubRight = iRight_.advertise("/camera/right/image_raw", 5);


    ImagePub ipub(strSeq, imgPubLeft, imgPubRight);

    ipub.PubliserImages();


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    tt2 = std::chrono::steady_clock::now();
    double t_round= std::chrono::duration_cast<std::chrono::duration<double> >(tt2 - tt1).count();
    //cout << "Round time cost = " <<  t_round  << "s,  freqency = " << 1/t_round << "Hz\n" << endl;
    tt1 = tt2;
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
     
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());

	    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
	    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
	    //double tall= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();

	    //cout << "Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
	    //cout << "Tracking time = " << ttrack << "s, freqency = " << 1/ttrack << "Hz" << endl; 	
	    //cout << "ALL cost time = " << tall << "s, freqency = " << 1/tall << "Hz" << endl; 
    }

}



void ImagePub::LoadImages()
{
    ifstream fTimes;
    string strPathTimeFile = strSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();
    vstrLeftImgFilenames.resize(nTimes);
    vstrRightImgFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrLeftImgFilenames[i] = strSequence + "/image_0/" + ss.str() + ".png";
        vstrRightImgFilenames[i] = strSequence + "/image_1/" + ss.str() + ".png";
    }

}


void ImagePub::PubliserImages()
{
    int cnt = 0;
    ros::Rate loop_rate(10);
    cv_bridge::CvImage cviLeft;
    cv_bridge::CvImage cviRight;

    cviLeft.header.frame_id = "image";
    cviLeft.encoding = "bgr8";

    cviRight.header.frame_id = "image";
    cviRight.encoding = "bgr8";

    LoadImages();
    
    while(ros::ok())
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cv::Mat imgLeft = cv::imread(vstrLeftImgFilenames[cnt]);
        
        if(imgLeft.empty())
        {
            cout << "image read error!" << endl;
            return ;
        }

        cv::Mat imgRight = cv::imread(vstrRightImgFilenames[cnt]);
        
        if(imgRight.empty())
        {
            cout << "image read error!" << endl;
            return ;
        }


        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        if(cnt % 5 == 0 && false)
        {
            cout << cnt << "  Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
            //cout<< cnt << endl;
        }

        //imshow("img",img);

        ros::Time time=ros::Time::now();

        cviLeft.header.stamp = time;
        cviLeft.image = imgLeft;

        sensor_msgs::Image imL;
        cviLeft.toImageMsg(imL);

        cviRight.header.stamp = time;
        cviRight.image = imgRight;

        sensor_msgs::Image imR;
        cviRight.toImageMsg(imR);

        leftImgPub_.publish(imL);
        rightImgPub_.publish(imR);
        
        cnt++;
        if(cnt>=vstrLeftImgFilenames.size())
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return;

}

