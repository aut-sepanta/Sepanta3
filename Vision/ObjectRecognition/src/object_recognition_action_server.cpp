#include <object_recognition_action_server.hpp>
#include <image_conversions.hpp>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>

ObjectRecognitionAction::ObjectRecognitionAction(ros::NodeHandle nh, std::string name) : node_handle(nh),
    action_name_(name),
    point_cloud_subscriber(node_handle, "/camera/depth_registered/points", 1),
    camera_info_subscriber(node_handle, "/camera/rgb/camera_info", 1),
    rgbd_image_synchronizer(RgbdImagePolicy(10), point_cloud_subscriber, camera_info_subscriber)
{
    boost::shared_ptr<std::vector<Object>> trained_objects(new std::vector<Object>);
    this->loadModels(trained_objects);
    this->object_pipeline = boost::shared_ptr<ObjectPipeline>(new ObjectPipeline(trained_objects));
    this->objects_publisher = node_handle.advertise<sepanta_msgs::Objects>("/object_recognition/objects", 5);

    action_server_.registerGoalCallback(boost::bind(&ObjectRecognitionAction::goalCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&ObjectRecognitionAction::preemptCallback, this));
    action_server_.start();
}

void ObjectRecognitionAction::goalCallback() {
    result_.objects.clear();
    point_cloud_subscriber.subscribe(node_handle, "/camera/depth_registered/points", 1);
    camera_info_subscriber.subscribe(node_handle, "/camera/rgb/camera_info", 1);
    action_server_.acceptNewGoal();
}

void ObjectRecognitionAction::preemptCallback() {
    point_cloud_subscriber.unsubscribe();
    camera_info_subscriber.unsubscribe();
    action_server_.setPreempted();
}

void ObjectRecognitionAction::rgbdImageCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                          const sensor_msgs::CameraInfoConstPtr& camera_info) {
    if (!action_server_.isActive()) return;

    camera_model.fromCameraInfo(camera_info);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *input_cloud);

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
        result_.objects.push_back(object_msg);
    }
    action_server_.setSucceeded(result_);
}
