
#include <ros/ros.h>
#include <ros/package.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

void save_params(libfreenect2::Freenect2Device::ColorCameraParams& color, libfreenect2::Freenect2Device::IrCameraParams& ir) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "color_camera_params";
    out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "f";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << color.fx << color.fy;
            out << YAML::EndSeq;
        out << YAML::Key << "c";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << color.cx << color.cy;
            out << YAML::EndSeq;
        out << YAML::Key << "shift";
            out << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "m" << YAML::Value << color.shift_m;
            out << YAML::Key << "d" << YAML::Value << color.shift_d;
            out << YAML::EndMap;
        out << YAML::Key << "Mx";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << color.mx_x3y0 << color.mx_x0y3 << color.mx_x2y1 << color.mx_x1y2 << color.mx_x2y0 << color.mx_x0y2 << color.mx_x1y1 << color.mx_x1y0 << color.mx_x0y1 << color.mx_x0y0;
            out << YAML::EndSeq;
        out << YAML::Key << "My";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << color.my_x3y0 << color.my_x0y3 << color.my_x2y1 << color.my_x1y2 << color.my_x2y0 << color.my_x0y2 << color.my_x1y1 << color.my_x1y0 << color.my_x0y1 << color.my_x0y0;
            out << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::Key << "ir_camera_params";
    out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "f";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << ir.fx << ir.fy;
            out << YAML::EndSeq;
        out << YAML::Key << "c";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << ir.cx << ir.cy;
            out << YAML::EndSeq;
        out << YAML::Key << "k";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << ir.k1 << ir.k2 << ir.k3;
            out << YAML::EndSeq;
        out << YAML::Key << "p";
            out << YAML::Value << YAML::Flow << YAML::BeginSeq;
            out << ir.p1 << ir.p2;
            out << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndMap;

    const char* yaml = out.c_str();
    ROS_INFO_STREAM("Read parameters from kinect\n" << yaml);   

    std::string save_path = ros::package::getPath("k2_client") + "/data/kinect2_params.yml";
    std::ofstream yaml_file(save_path.c_str(), std::ofstream::out);
    yaml_file << yaml;
    yaml_file.close();
    ROS_INFO_STREAM("Parameters saved to: " << save_path);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "grab_kinect_params");

    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::None));
    libfreenect2::Freenect2 freenect2;
    if (freenect2.enumerateDevices() == 0) {
        ROS_FATAL("No kinect device found");
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    libfreenect2::Freenect2Device *dev = freenect2.openDevice(serial);
    
    if (dev == 0) {
        ROS_INFO("Cannot connect to kinect");
    }

    ROS_INFO_STREAM("Connected to kinect serial=" << dev->getSerialNumber() << ", firmware=" << dev->getFirmwareVersion());

    int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir;

    dev->start();
    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();

    save_params(color_params, ir_params);

    dev->stop();
    dev->close();

    return 0;
}
