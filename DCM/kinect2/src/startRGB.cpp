/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -    Redistributions of source code must retain the above copyright notice, this list 
     of conditions and the following disclaimer.
 -    Redistributions in binary form must reproduce the above copyright notice, this 
     list of conditions and the following disclaimer in the documentation and/or other 
     materials provided with the     distribution.
 -    Neither the name of Carnegie Mellon University nor the names of its contributors 
     may be used to endorse or promote products derived from this software without 
     specific prior written     permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client.h"
#include <tf/transform_broadcaster.h>

int imageSize = 2764800;
int streamSize = 2768640;// imageSize + sizeof(double);
std::string cameraName = "rgb";
std::string imageTopicSubName = "image_color";
std::string cameraInfoSubName = "camera_info";

int main(int argC,char **argV)
{
    ros::init(argC,argV,"startRGB");
    ros::NodeHandle n(cameraName);
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    std::vector<unsigned char> bufferVector(streamSize);
    unsigned char *buffer = &bufferVector[0];
    Socket mySocket(serverAddress.c_str(),"9000",(char*)buffer,imageSize+sizeof(double));
    ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>(imageTopicSubName,1);
    ros::Publisher cameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(cameraInfoSubName,1);
    camera_info_manager::CameraInfoManager camInfoMgr(n,cameraName);
    camInfoMgr.loadCameraInfo("");
    cv::Mat frame;
    cv_bridge::CvImage cvImage;
    sensor_msgs::Image rosImage;

    tf::TransformBroadcaster broadcaster;
    tf::Matrix3x3 rot(9.9979883926593693e-01, -1.6871335541758126e-02, -1.0846153213119176e-02,
                       1.6891097527572209e-02,  9.9985583336600126e-01, 1.7330055664057141e-03,
                       1.0815351441312036e-02, -1.9158603854556545e-03, 9.9993967700666042e-01);
    tf::Vector3 trans(-5.2052476112081990e-02, -4.6313865353939110e-04, 8.8806735554907584e-04);
    tf::Transform tDepth(rot, trans);

    tf::Vector3 vZero(0, 0, 0);
    tf::Quaternion qZero;
    qZero.setRPY(0, 0, 0);
    tf::Transform tZero(qZero, vZero);

    tf::StampedTransform stColor, stDepth;

    stColor = tf::StampedTransform(tZero, ros::Time::now(), "kinect2_frame", "rgb_image_frame");
    stDepth = tf::StampedTransform(tDepth, ros::Time::now(), "rgb_image_frame", "depth_image_frame");
    int seq = 0;
    while(ros::ok())
    {
        mySocket.readData();
        double utcTime;
        memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
        rosImage.header.seq = seq++;
        rosImage.header.stamp = ros::Time(utcTime);
        rosImage.header.frame_id = "rgb_image_frame";
        rosImage.encoding = "rgb8";
        rosImage.width = 1280;
        rosImage.height = 721;
        rosImage.step = 3840; // = 1920*3bytes
        rosImage.data = bufferVector;
        sensor_msgs::CameraInfo camInfo = camInfoMgr.getCameraInfo();
        camInfo.header.stamp = cvImage.header.stamp;
        camInfo.header.frame_id = cvImage.header.frame_id;
        cameraInfoPub.publish(camInfo);
        imagePublisher.publish(rosImage);

        stColor.stamp_ = rosImage.header.stamp;
        stDepth.stamp_ = rosImage.header.stamp;
        broadcaster.sendTransform(stColor);
        broadcaster.sendTransform(stDepth);

        ros::spinOnce();
    }
    return 0;
}
