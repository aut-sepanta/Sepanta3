#ifndef _TRAIN_OBJECTS_HPP
#define _TRAIN_OBJECTS_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <object_pipeline.hpp>

class TrainObjects {
public:
    TrainObjects();

    void rgbdImageCallback(const sensor_msgs::ImageConstPtr& input_image, const sensor_msgs::PointCloud2ConstPtr& input_cloud);
    void pipeline(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
    void saveModel(boost::shared_ptr<std::vector<Object>> objects);
private:
    void cliThread(boost::shared_ptr<std::vector<Object>> objects);

    ObjectPipeline object_pipeline;
    bool trainingFinished;
};

#endif
