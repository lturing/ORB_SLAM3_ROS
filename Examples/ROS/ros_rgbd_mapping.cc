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
#include<thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){};

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:

    ORB_SLAM3::System* mpSLAM;

};

class ImagePub
{
    // for kitti dataset
    public:
        ImagePub(ORB_SLAM3::System* pSLAM, const string& strSequence, image_transport::Publisher& imgPub, image_transport::Publisher& depPub)
            :strSequence(strSequence), imgPub_(imgPub), depPub_(depPub), mpSLAM(pSLAM) {}

        void LoadImages();
        void PubliserImages();
        
        string strSequence;
        image_transport::Publisher imgPub_, depPub_;
        vector<string> vstrImgFilenames, vstrDepFilenames;
        vector<double> vTimestamps;
        ORB_SLAM3::System* mpSLAM;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_mapping");
    ros::start();
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::Time::init();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    tf::TransformListener listener;

    string voc_dir = argv[2];
    string config_dir = argv[1];
    string strSeq = argv[3];
    
    // get parameters
    bool use_rviz;
    private_nh.param("use_rviz", use_rviz, true);

    ORB_SLAM3::System SLAM(voc_dir,config_dir,ORB_SLAM3::System::RGBD,!use_rviz);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    image_transport::ImageTransport img_(nh);
    image_transport::Publisher imgPub;
    imgPub = img_.advertise("/camera/rgb/image_color", 1);

    image_transport::ImageTransport dep_(nh);
    image_transport::Publisher depPub;
    depPub = dep_.advertise("/camera/depth/image", 1);

    ImagePub ipub(&SLAM, strSeq, imgPub, depPub);

    ipub.PubliserImages();

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s in image", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::MONO16); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s in depth", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

}


void ImagePub::LoadImages()
{
    ifstream ff;
    string strPathTimeFile = strSequence + "/associate.txt";
    ff.open(strPathTimeFile.c_str());
    while(!ff.eof())
    {
        string s;
        getline(ff,s);
        if(!s.empty())
        {            
            istringstream iss(s);   
            string token;           
            getline(iss, token, ' ');
            double timestamp = std::stod(token);
            vTimestamps.push_back(timestamp);

            getline(iss, token, ' ');

            vstrImgFilenames.push_back(strSequence + "/" + token);

            //cout << (strSequence + "/" + token) << endl;

            getline(iss, token, ' ');

            getline(iss, token, ' ');

            vstrDepFilenames.push_back(strSequence + "/" + token);

            //cout << (strSequence + "/" + token) << endl;

        }
    }

}


void ImagePub::PubliserImages()
{
    int cnt = 0;
    ros::Rate loop_rate(10);
    cv_bridge::CvImage cviImg;
    cv_bridge::CvImage cviDepth;

    cviImg.header.frame_id = "image";
    cviImg.encoding = "bgr8";

    cviDepth.header.frame_id = "image";
    cviDepth.encoding = "mono16";  //sensor_msgs::image_encodings::MONO16;

    LoadImages();
    
    while(ros::ok())
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cv::Mat img = cv::imread(vstrImgFilenames[cnt], cv::IMREAD_UNCHANGED);
        
        if(img.empty())
        {
            cout << "image read error! in img" << endl;
            cout << vstrImgFilenames[cnt] << " " << vstrDepFilenames[cnt] << endl;
            return ;
        }

        cv::Mat depth = cv::imread(vstrDepFilenames[cnt], cv::IMREAD_UNCHANGED);
        
        //cout << "img.type()=" << img.type() << ", depth.size()=" << depth.size() << ", depth.type()=" << depth.type() << ", depth.channels()=" << depth.channels() << endl; 
        //cout << sensor_msgs::image_encodings::MONO16 << endl; 

        if(depth.empty())
        {
            cout << "image read error! in depth" << endl;
            cout << vstrImgFilenames[cnt] << " " << vstrDepFilenames[cnt] << endl;
            return ;
        }

        /*
        if(mpSLAM->GetImageScale() != 1.f)
        {
            int width = img.cols * mpSLAM->GetImageScale();
            int height = img.rows * mpSLAM->GetImageScale();
            cv::resize(img, img, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }
        */ 

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        if(cnt % 5 == 0 && false)
        {
            cout << cnt << "  Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
            //cout<< cnt << endl;
        }

        //imshow("img",img);

        ros::Time time=ros::Time::now();

        cviImg.header.stamp = time;
        cviImg.image = img;

        sensor_msgs::Image imL;
        cviImg.toImageMsg(imL);

        cviDepth.header.stamp = time;
        cviDepth.image = depth;

        sensor_msgs::Image imR;
        cviDepth.toImageMsg(imR);

        imgPub_.publish(imL);
        depPub_.publish(imR);
        
        cnt++;
        if(cnt>=vstrImgFilenames.size())
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return;

}


