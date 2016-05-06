#ifndef IMAGE_CONVERSIONS_HPP
#define IMAGE_CONVERSIONS_HPP

#include <geometry_msgs/Point.h>
#include <image_geometry/pinhole_camera_model.h>

namespace image_conversions {

void PointXYZtoCameraPointXY(const geometry_msgs::Point input, geometry_msgs::Point &output, const image_geometry::PinholeCameraModel& model);

}

#endif
