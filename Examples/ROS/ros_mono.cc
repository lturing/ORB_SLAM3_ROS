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



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>


#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <sstream>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "System.h"
#include<opencv2/core/core.hpp>



using namespace std;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


class ImagePub
{
    // for kitti dataset
    public:
        ImagePub(const string& strSequence, image_transport::Publisher& imgPub):strSequence(strSequence), image_pub_(imgPub) {}

        void LoadImages();
        void PubliserImages();
        
        string strSequence;
        image_transport::Publisher image_pub_;
        vector<string> vstrImageFilenames;
        vector<double> vTimestamps;

};


long spinCnt=0;
double t_temp=0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }    
    
    string voc_dir = argv[2];
    string config_dir = argv[1];
    string strSeq = argv[3];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_dir,config_dir,ORB_SLAM3::System::MONOCULAR,false);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    
    image_transport::ImageTransport it_(nodeHandler);
    image_transport::Publisher image_pub;
    image_pub = it_.advertise("/camera/image_raw", 5);
    ImagePub ipub(strSeq, image_pub);

    ipub.PubliserImages();
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    /*
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double tall= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
    
    cout << "Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
    cout << "Tracking time = " << ttrack << "s, freqency = " << 1/ttrack << "Hz" << endl; 
    cout << "ALL cost time = " << tall << "s, freqency = " << 1/tall << "Hz\n" << endl; 
    t_temp = (tall + t_temp*spinCnt)/(1+spinCnt);
    cout << "Avg. time = " << t_temp << "s, freqency = " << 1/t_temp << "Hz\n" << endl; 
    spinCnt++;
    */
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

    string strPrefixLeft = strSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }

}


void ImagePub::PubliserImages()
{
    int cnt = 0;
    ros::Rate loop_rate(10);
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    LoadImages();
    
    while(ros::ok())
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cv::Mat img = cv::imread(vstrImageFilenames[cnt]);
        
        if(img.empty())
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

        cvi.header.stamp = time;
        cvi.image = img;

        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        image_pub_.publish(im);
        
        cnt++;
        if(cnt>=vstrImageFilenames.size())
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return;

}

