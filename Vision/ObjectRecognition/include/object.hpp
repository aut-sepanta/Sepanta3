#ifndef _OBJECT_HPP
#define _OBJECT_HPP

#include <pcl/common/common.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <string>

class Object {
public:
    Object();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors;
    pcl::CorrespondencesPtr correspondences;
    std::string label;
    double cloud_resolution;
};

#endif
