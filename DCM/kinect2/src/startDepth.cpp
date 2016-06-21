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

int imageSize = 434176;
int streamSize = 435200;// imageSize + sizeof(double);
std::string cameraName = "depth";
//std::string imageTopicSubName = "/kinect2/sd/image_depth_rect";
//std::string cameraInfoSubName = "/kinect2/sd/image_depth_rect/camera_info";
std::string imageTopicSubName = "/kinect2/depth/image";
std::string cameraInfoSubName = "/kinect2/sd/image_depth_rect/camera_info";

int main(int argC,char **argV)
{
    ros::init(argC,argV,"startDepth");
    ros::NodeHandle n(cameraName);
    std::string serverAddress;
    n.getParam("/serverNameOrIP",serverAddress);
    std::vector<unsigned char> bufferVector(streamSize);
    unsigned char *buffer = &bufferVector[0];
    Socket mySocket(serverAddress.c_str(),"9001",(char*)buffer,imageSize+sizeof(double));
    ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>(imageTopicSubName,1);
    ros::Publisher cameraInfoPub = n.advertise<sensor_msgs::CameraInfo>(cameraInfoSubName,1);
    camera_info_manager::CameraInfoManager camInfoMgr(n,cameraName);
    camInfoMgr.loadCameraInfo("");
    sensor_msgs::Image rosImage;
    int seq = 0;
    while(ros::ok())
    {
        mySocket.readData(false);
        double utcTime;
        memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
        rosImage.header.frame_id = "depth_image_frame";
        rosImage.header.seq = seq++;
        rosImage.header.stamp = ros::Time(utcTime);
        rosImage.encoding = "mono16";
        rosImage.width = 512;
        rosImage.height = 425;
        rosImage.step = 1024; // = 1920*3bytes
        rosImage.data = bufferVector;
        sensor_msgs::CameraInfo camInfo = camInfoMgr.getCameraInfo();
        camInfo.header.stamp = ros::Time(utcTime);
        camInfo.header.frame_id = rosImage.header.frame_id;
        cameraInfoPub.publish(camInfo);
        imagePublisher.publish(rosImage);
        ros::spinOnce();
    }
    return 0;
}
