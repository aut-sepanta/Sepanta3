#ifndef _OBJECT_HPP
#define _OBJECT_HPP

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

class Object {
public:
    Object();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
    pcl::PointCloud<pcl::SHOT1344>::Ptr descriptors;
    double cloud_resolution;
};

#endif
