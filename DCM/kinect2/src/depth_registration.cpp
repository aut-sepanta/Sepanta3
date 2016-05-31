
#include <ros/ros.h>
#include <ros/package.h>
#include <libfreenect2/libfreenect2.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <yaml-cpp/yaml.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

std::string colorTopicName = "/kinect2/hd/image_color_rect";
std::string depthTopicName = "/kinect2/sd/image_depth_rect";
std::string cameraInfoPostfix = "/camera_info";

float depth_q = 0.01;
float color_q = 0.002199;

libfreenect2::Freenect2Device::IrCameraParams ir_params;
libfreenect2::Freenect2Device::ColorCameraParams color_params;

float rx_lut[512*424];
float ry_lut[512*424];

void depth_to_color(int dx, int dy, float dz, float& cx, float &cy) 
{
  const int index = dx + dy * 512;
  float rx = rx_lut[index];
  cy = ry_lut[index];

  rx += (52.0 / dz);
  cx = rx * color_params.fx + color_params.cx;
}

void create_luts(libfreenect2::Freenect2Device::ColorCameraParams &params, libfreenect2::Freenect2Device::IrCameraParams &ir_params) {
    for (unsigned int y=0; y<424; y++) {
    	for (unsigned int x=0; x<512; x++) {
            float dx = ((float)x - ir_params.cx) / ir_params.fx;
            float dy = ((float)y - ir_params.cy) / ir_params.fy;
            float dx2 = dx * dx;
            float dy2 = dy * dy;
            float r2 = dx2 + dy2;
            float dxdy2 = 2 * dx * dy;
            float kr = 1 + ((ir_params.k3 * r2 + ir_params.k2) * r2 + ir_params.k1) * r2;
            float mx = ir_params.fx * (dx * kr + ir_params.p2 * (r2 + 2 * dx2) + ir_params.p1 * dxdy2) + ir_params.cx;
            float my = ir_params.fy * (dy * kr + ir_params.p1 * (r2 + 2 * dy2) + ir_params.p2 * dxdy2) + ir_params.cy;

            rx_lut[x+y*512] = mx; 
            ry_lut[x+y*512] = my;

            mx = (mx - ir_params.cx) * depth_q;
            my = (my - ir_params.cy) * depth_q;
            float wx =
                (mx * mx * mx * params.mx_x3y0) + (my * my * my * params.mx_x0y3) +
                (mx * mx * my * params.mx_x2y1) + (my * my * mx * params.mx_x1y2) +
                (mx * mx * params.mx_x2y0) + (my * my * params.mx_x0y2) + (mx * my * params.mx_x1y1) +
                (mx * params.mx_x1y0) + (my * params.mx_x0y1) + (params.mx_x0y0);
            float wy =
                (mx * mx * mx * params.my_x3y0) + (my * my * my * params.my_x0y3) +
                (mx * mx * my * params.my_x2y1) + (my * my * mx * params.my_x1y2) +
                (mx * mx * params.my_x2y0) + (my * my * params.my_x0y2) + (mx * my * params.my_x1y1) +
                (mx * params.my_x1y0) + (my * params.my_x0y1) + (params.my_x0y0);

            rx_lut[x+y*512] = (wx / (params.fx * color_q)) - (params.shift_m / params.shift_d);
            ry_lut[x+y*512] = (wy / color_q) + params.cy;
        }
    }
}


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> RgbdImagePolicy;

void load_kinect_params() {
    std::string load_path = ros::package::getPath("k2_client") + "/data/kinect2_params.yml";
    YAML::Node yaml = YAML::LoadFile(load_path);
    
    color_params.mx_x3y0 = yaml["color_camera_params"]["Mx"][0].as<float>();
    color_params.mx_x0y3 = yaml["color_camera_params"]["Mx"][1].as<float>();
    color_params.mx_x2y1 = yaml["color_camera_params"]["Mx"][2].as<float>();
    color_params.mx_x1y2 = yaml["color_camera_params"]["Mx"][3].as<float>();
    color_params.mx_x2y0 = yaml["color_camera_params"]["Mx"][4].as<float>();
    color_params.mx_x0y2 = yaml["color_camera_params"]["Mx"][5].as<float>();
    color_params.mx_x1y1 = yaml["color_camera_params"]["Mx"][6].as<float>();
    color_params.mx_x1y0 = yaml["color_camera_params"]["Mx"][7].as<float>();
    color_params.mx_x0y1 = yaml["color_camera_params"]["Mx"][8].as<float>();
    color_params.mx_x0y0 = yaml["color_camera_params"]["Mx"][9].as<float>();
    
    color_params.my_x3y0 = yaml["color_camera_params"]["My"][0].as<float>();
    color_params.my_x0y3 = yaml["color_camera_params"]["My"][1].as<float>();
    color_params.my_x2y1 = yaml["color_camera_params"]["My"][2].as<float>();
    color_params.my_x1y2 = yaml["color_camera_params"]["My"][3].as<float>();
    color_params.my_x2y0 = yaml["color_camera_params"]["My"][4].as<float>();
    color_params.my_x0y2 = yaml["color_camera_params"]["My"][5].as<float>();
    color_params.my_x1y1 = yaml["color_camera_params"]["My"][6].as<float>();
    color_params.my_x1y0 = yaml["color_camera_params"]["My"][7].as<float>();
    color_params.my_x0y1 = yaml["color_camera_params"]["My"][8].as<float>();
    color_params.my_x0y0 = yaml["color_camera_params"]["My"][9].as<float>();

    color_params.shift_d = yaml["color_camera_params"]["shift"]["d"].as<float>();
    color_params.shift_m = yaml["color_camera_params"]["shift"]["m"].as<float>();

    color_params.cx = yaml["color_camera_params"]["c"][0].as<float>();
    color_params.cy = yaml["color_camera_params"]["c"][1].as<float>();

    color_params.fx = yaml["color_camera_params"]["f"][0].as<float>();
    color_params.fy = yaml["color_camera_params"]["f"][1].as<float>();

    ir_params.fx = yaml["ir_camera_params"]["f"][0].as<float>();
    ir_params.fy = yaml["ir_camera_params"]["f"][1].as<float>();

    ir_params.cx = yaml["ir_camera_params"]["c"][0].as<float>();
    ir_params.cy = yaml["ir_camera_params"]["c"][1].as<float>();

    ir_params.p1 = yaml["ir_camera_params"]["p"][0].as<float>();
    ir_params.p2 = yaml["ir_camera_params"]["p"][1].as<float>();

    ir_params.k1 = yaml["ir_camera_params"]["k"][0].as<float>();
    ir_params.k2 = yaml["ir_camera_params"]["k"][1].as<float>();
    ir_params.k3 = yaml["ir_camera_params"]["k"][2].as<float>();
}

ros::Publisher pcl_pub;
ros::Publisher rgb_pub;

void rgbdImageCallback(const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::ImageConstPtr& depth_image,
                       const sensor_msgs::CameraInfoConstPtr& rgb_cam_info, const sensor_msgs::CameraInfoConstPtr& depth_cam_info)
{
    ROS_INFO_STREAM("CALL");
    float scale_factor = 1280.f/1920.f;
    float x_world, y_world, z_world;
    float x_color, y_color;
    unsigned char r, g, b;
    unsigned int step = rgb_image->step;

    sensor_msgs::Image rosImage;
   
    std::vector<unsigned char> bufferVector(2768640);
    unsigned char *buffer = &bufferVector[0];

    for (int y=0; y < 721; y++) {
        for (int x=0; x < 1280; x++) {
            buffer[3*(x+y*1280)] = rgb_image->data[3*(x+y*1280)];
            buffer[3*(x+y*1280)+1] = rgb_image->data[3*(x+y*1280)+1];
            buffer[3*(x+y*1280)+2] = rgb_image->data[3*(x+y*1280)+2];
        }
    }
    PointCloud::Ptr msg (new PointCloud);
    for (int y=0; y < 424; y++) {
        for (int x=0; x < 512; x++) {
            depth_to_color(x, y, depth_image->data[x+512*y], x_color, y_color);

            x_color *= scale_factor;
            y_color *= scale_factor;
            if (x_color<0 || x_color>=rgb_image->width || y_color<0 || y_color>=rgb_image->height)
                continue;

//            ROS_INFO_STREAM("C(" << x_color << ", " << y_color << ", " << step*(unsigned int)y_color + (unsigned int)x_color << ")");
            z_world = depth_image->data[x+512*y];
            x_world = (x-color_params.cx) * z_world / color_params.fx;
            y_world = (y-color_params.cy) * z_world / color_params.fy;

            buffer[3*((unsigned int)y_color*1280 + (unsigned int)x_color)] = 0;
            buffer[3*((unsigned int)y_color*1280 + (unsigned int)x_color)+1] = 0;
            buffer[3*((unsigned int)y_color*1280 + (unsigned int)x_color)+2] = depth_image->data[x+512*y];
            r = rgb_image->data[3*((unsigned int)y_color*512 + (unsigned int)x_color)];
            g = rgb_image->data[3*((unsigned int)y_color*512 + (unsigned int)x_color)+1];
            b = rgb_image->data[3*((unsigned int)y_color*512 + (unsigned int)x_color)+2];

            //FIXME: msg->header.stamp = depth_image->header.stamp;
            msg->header.frame_id = rgb_image->header.frame_id;
            pcl::PointXYZRGB point(r,g,b);
            //ROS_INFO_STREAM(x_world << " " << y_world << " " << z_world);
            point.x = x_world;
            point.y = y_world;
            point.z = z_world;
            msg->points.push_back(point);
        }
    }
    rosImage.header.seq = rgb_image->header.seq;
    rosImage.header.stamp = ros::Time::now();
    rosImage.header.frame_id = "rgb_image_frame";
    rosImage.encoding = "rgb8";
    rosImage.width = 1280;
    rosImage.height = 721;
    rosImage.step = 3840; // = 1920*3bytes
    ROS_INFO_STREAM("size=" << bufferVector.size());
    rosImage.data = bufferVector;

    rgb_pub.publish(rosImage);
    ROS_INFO_STREAM("PUB");
    pcl_pub.publish(msg);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "depth_registration");
    ros::NodeHandle node_handle;
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_subscriber(node_handle, colorTopicName, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_cam_info_subscriber(node_handle, colorTopicName + cameraInfoPostfix, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_image_subscriber(node_handle, depthTopicName, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_cam_info_subscriber(node_handle, depthTopicName + cameraInfoPostfix, 1);
    message_filters::Synchronizer<RgbdImagePolicy> rgbd_image_synchronizer(RgbdImagePolicy(10), rgb_image_subscriber, depth_image_subscriber,
                                                                           rgb_cam_info_subscriber, depth_cam_info_subscriber);

    load_kinect_params();
    create_luts(color_params, ir_params);
    pcl_pub = node_handle.advertise<PointCloud> ("/kinect2/points_registered", 1);
    rgb_pub = node_handle.advertise<sensor_msgs::Image> ("/kinect2/rgb_reg", 1);
    rgbd_image_synchronizer.registerCallback(rgbdImageCallback);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
