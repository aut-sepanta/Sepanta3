#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sepanta_msgs/Objects.h>
#include <std_srvs/Empty.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <object_recognition.hpp>
#include <object_pipeline.hpp>
#include <image_conversions.hpp>

ObjectRecognition::ObjectRecognition(ros::NodeHandle node_handle) :
    node_handle(node_handle),
    point_cloud_subscriber(node_handle, "/camera/depth_registered/points", 1),
    rgb_image_subscriber(node_handle, "/camera/rgb/image_color", 1),
    camera_info_subscriber(node_handle, "/camera/rgb/camera_info", 1),
    rgbd_image_synchronizer(RgbdImagePolicy(10), rgb_image_subscriber, point_cloud_subscriber, camera_info_subscriber)
{

    boost::shared_ptr<std::vector<Object>> trained_objects(new std::vector<Object>);
    this->loadModels(trained_objects);
    this->object_pipeline = boost::shared_ptr<ObjectPipeline>(new ObjectPipeline(trained_objects));

    this->objects_publisher = node_handle.advertise<sepanta_msgs::Objects>("/object_recognition/objects", 5);
    this->turnOnService = node_handle.advertiseService("/object_recognition/turn_on", &ObjectRecognition::turnOn, this);
    this->turnOffService = node_handle.advertiseService("/object_recognition/turn_off", &ObjectRecognition::turnOff, this);
}

bool ObjectRecognition::turnOff(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    point_cloud_subscriber.unsubscribe();
    rgb_image_subscriber.unsubscribe();
    camera_info_subscriber.unsubscribe();
    return true;
}


bool ObjectRecognition::turnOn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    point_cloud_subscriber.subscribe(node_handle, "/camera/depth_registered/points", 1);
    rgb_image_subscriber.subscribe(node_handle, "/camera/rgb/image_color", 1);
    camera_info_subscriber.subscribe(node_handle, "/camera/rgb/camera_info", 1);
    rgbd_image_synchronizer.registerCallback(boost::bind(&ObjectRecognition::rgbdImageCallback, this, _1, _2, _3));
    return true;
}

void ObjectRecognition::rgbdImageCallback(const sensor_msgs::ImageConstPtr& input_image, const sensor_msgs::PointCloud2ConstPtr& input_cloud,
                                          const sensor_msgs::CameraInfoConstPtr& camera_info) {
    camera_model.fromCameraInfo(camera_info);

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

void ObjectRecognition::publish(boost::shared_ptr<std::vector<Object>> objects) {
    sepanta_msgs::ObjectsPtr objects_msg(new sepanta_msgs::Objects);

    for (unsigned int i=0; i<objects->size(); i++) {
        sepanta_msgs::Object object_msg;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*(objects->at(i).cloud), centroid);
        geometry_msgs::Pose object_pose;
        object_pose.position.x = centroid(0);
        object_pose.position.y = centroid(1);
        object_pose.position.z = centroid(2);
        object_pose.orientation.x = 0.0;
        object_pose.orientation.y = 0.0;
        object_pose.orientation.z = 0.0;
        object_pose.orientation.w = 1.0;
        object_msg.pose = object_pose;

        image_conversions::PointXYZtoCameraPointXY(object_pose.position, object_msg.center_2d, camera_model);

        object_msg.validity = objects->at(i).validity;
        object_msg.label = objects->at(i).label;
        object_msg.status = objects->at(i).label.compare("unknown") ? sepanta_msgs::Object::STATUS_UNKNOWN :
                                                                     sepanta_msgs::Object::STATUS_RECOGNIZED;
        objects_msg->objects.push_back(object_msg);
    }
    this->objects_publisher.publish(objects_msg);
}

void ObjectRecognition::pipeline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    boost::shared_ptr<std::vector<Object>> objects;
    filtered_cloud = this->object_pipeline->passthroughPointCloud(input_cloud);
    if (nullptr == filtered_cloud) {
        return;
    }
    table_hull = this->object_pipeline->findTableHull(filtered_cloud);
    if (nullptr == table_hull) {
        return;
    }
    objects_cloud = this->object_pipeline->createObjectCloud(filtered_cloud, table_hull);
    if (nullptr == objects_cloud) {
        return;
    }
    objects = this->object_pipeline->createObjectClusters(objects_cloud);
    this->object_pipeline->keyPointExtraction(objects);
    this->object_pipeline->createObjectDescriptors(objects);
    this->object_pipeline->objectMatching(objects);
    this->publish(objects);
}

void ObjectRecognition::loadModels(boost::shared_ptr<std::vector<Object>> objects) {
    boost::filesystem::path models_path(ros::package::getPath("object_recognition") + "/trained_objects");
    boost::regex modelfile_pattern("^model_(\D+)\d*[.]pcd$");
    for (boost::filesystem::recursive_directory_iterator iter(models_path), end; iter!=end; iter++) {
        boost::match_results<std::string::const_iterator> results;
        std::string file_path = iter->path().string(); 
        std::string file_name = iter->path().leaf().string(); 
        if (regex_match(file_name, results, modelfile_pattern)) {
            std::string object_name = std::string(results[1].first, results[1].second);
            Object trained_object;
            trained_object.label = object_name;
            if (-1 == pcl::io::loadPCDFile(file_path, *(trained_object.descriptors))) {
                ROS_ERROR_STREAM("Unable to load object model: \"" << file_path << "\"");
                continue;
            }
            objects->push_back(trained_object);
            ROS_INFO_STREAM("Loaded object model \"" << object_name << "\"");
        }
    }
}
