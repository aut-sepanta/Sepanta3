
#include <object.hpp>

Object::Object() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
                 normals(new pcl::PointCloud<pcl::PointNormal>),
                 keypoints(new pcl::PointCloud<pcl::PointXYZRGB>),
                 descriptors(new pcl::PointCloud<pcl::SHOT1344>) {
}
