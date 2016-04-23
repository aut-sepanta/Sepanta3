/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list 
 	of conditions and the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this 
 	list of conditions and the following disclaimer in the documentation and/or other 
 	materials provided with the 	distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors 
 	may be used to endorse or promote products derived from this software without 
 	specific prior written 	permission.
 
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

int imageSize = 6220800;
int streamSize = 6226560;// imageSize + sizeof(double);
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
    int seq = 0;
	while(ros::ok())
	{
		mySocket.readData();
		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[imageSize],sizeof(double));
        rosImage.header.seq = seq++;
        rosImage.header.stamp = ros::Time(utcTime);
        rosImage.header.frame_id = ros::this_node::getNamespace() + "/colorFrame"; 
		rosImage.encoding = "rgb8";
        rosImage.width = 1920;
        rosImage.height = 1081;
        rosImage.step = 5760; // = 1920*3bytes
        rosImage.data = bufferVector;
		sensor_msgs::CameraInfo camInfo = camInfoMgr.getCameraInfo();
		camInfo.header.stamp = cvImage.header.stamp;
		camInfo.header.frame_id = cvImage.header.frame_id;
		cameraInfoPub.publish(camInfo);
		imagePublisher.publish(rosImage);
		ros::spinOnce();
	}
	return 0;
}
