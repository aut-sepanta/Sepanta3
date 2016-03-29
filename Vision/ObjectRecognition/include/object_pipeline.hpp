#ifndef _OBJECT_PIPELINE
#define _OBJECT_PIPELINE

#include <pcl/point_types.h>

#include <object.hpp>

class ObjectPipeline {
public:
    ObjectPipeline();
    ObjectPipeline(boost::shared_ptr<std::vector<Object>> trained_objects);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr findTableHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createObjectCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_hull);
    boost::shared_ptr<std::vector<Object>> createObjectClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects);
    void keyPointExtraction(boost::shared_ptr<std::vector<Object>> objects);
    void createObjectDescriptors(boost::shared_ptr<std::vector<Object>> objects);
    void objectMatching(boost::shared_ptr<std::vector<Object>> objects);
    void correspondanceGrouping();
    void absoluteOrientation();
    void icpRefinement();
    void hypothesisVerification();
private:
    void computeCloudResolution (Object* object);
    boost::shared_ptr<std::vector<Object>> trained_objects;
};

#endif
