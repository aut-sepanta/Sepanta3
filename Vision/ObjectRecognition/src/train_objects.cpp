#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <train_objects.hpp>
#include <object_pipeline.hpp>

TrainObjects::TrainObjects() : trainingFinished(false) { 
}

void TrainObjects::rgbdImageCallback(const sensor_msgs::ImageConstPtr& input_image, const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
    if (input_image->width == 0 || input_cloud->width == 0) {
        ROS_WARN("image or point cloud is empty");
        return;
    }

    cv::Mat rgb_image;
    cv_bridge::CvImagePtr cv_bridge_image;
    try {
        cv_bridge_image = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
        if (cv_bridge_image->image.size().width > 0) 
        {
            cv_bridge_image->image.copyTo(rgb_image);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    this->pipeline(pcl_cloud);
}

void TrainObjects::pipeline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    boost::shared_ptr<std::vector<Object>> objects;
    filtered_cloud = this->object_pipeline.passthroughPointCloud(input_cloud);
    if (nullptr == filtered_cloud) {
        return;
    }
    table_hull = this->object_pipeline.findTableHull(filtered_cloud);
    if (nullptr == table_hull) {
        return;
    }
    objects_cloud = this->object_pipeline.createObjectCloud(filtered_cloud, table_hull);
    if (nullptr == objects_cloud) {
        return;
    }
    objects = this->object_pipeline.createObjectClusters(objects_cloud);
    this->object_pipeline.keyPointExtraction(objects);
    this->object_pipeline.createObjectDescriptors(objects);

    this->saveModel(objects);
}

void TrainObjects::saveModel(boost::shared_ptr<std::vector<Object>> objects) {
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    for (unsigned int i=0; i<objects->size(); i++) { 
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB>>(*(objects->at(i).cloud));
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr keypoints = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB>>(*(objects->at(i).keypoints));

        ROS_INFO_STREAM("Cloud size: " << cloud->size());
        ROS_INFO_STREAM("Keypoints size: " << keypoints->size());
        std::stringstream s1, s2, s3;
        s1 << "object cloud #" << i+1;
        s2 << "keypoints cloud #" << i+1;
        s3 << "cloud #" << i+1;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB> (cloud, cloud_rgb, s1.str());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, s1.str());
        viewer.addText3D(s3.str(), keypoints->back(), 0.05, 1, 1, 0, s3.str());

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (keypoints, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZRGB> (keypoints, single_color, s2.str());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, s2.str());
    }
    boost::thread cli_thread(boost::bind(&TrainObjects::cliThread, this, objects));
    
    while (!viewer.wasStopped() && !trainingFinished)
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    cli_thread.join();

}

void TrainObjects::cliThread(boost::shared_ptr<std::vector<Object>> objects) {
    for (unsigned int i=0; i<objects->size(); i++) {
        std::stringstream cloud_label;
        std::string chosen_name;
        cloud_label << "cloud #" << i+1;
        std::cout << "select a name for " << cloud_label.str() << "(d for delete): ";
        std::cin >> chosen_name;
        if (chosen_name.compare("d") == 0) {
            continue;
        }
        std::stringstream filename;
        std::string save_path = ros::package::getPath("object_recognition") + "/trained_objects/";
        filename << save_path << "model_" << chosen_name << ".pcd";
        pcl::io::savePCDFileBinary(filename.str(), *(objects->at(i).descriptors));   
    }

    trainingFinished = true;

    std::string choose;
    while (choose.compare("y") && choose.compare("n")) {
        std::cout << "Do you want to train more objects (y/n)?";
        std::cin >> choose;
    }

    if (!choose.compare("n")) {
        ros::shutdown();
    }

    trainingFinished = false;
}

