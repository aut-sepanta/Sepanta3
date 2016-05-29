
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "std_srvs/Empty.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_srvs/Empty.h"


//#define HAVE_NEW_YAMLCPP

#ifdef HAVE_NEW_YAMLCPP

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapGenerator 
{
    public:
    MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
    {
      ros::NodeHandle n,n2;
      ROS_INFO("Waiting for the map & Service v 1.0");
      map_sub_ = n.subscribe("/map", 1, &MapGenerator::mapCallback, this);

      get_map = false;
      service = n2.advertiseService("sepantamapengenine/save", &MapGenerator::checkcommand , this);
    }

   nav_msgs::OccupancyGridConstPtr map;
   ros::ServiceServer service;

   bool checkcommand(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
   {

      ROS_INFO("Save Service Request....");

    if ( get_map == false ) 
    { 
      ROS_ERROR("no map received yet");
    }
    else
    {
      ROS_INFO("Saving...");
      save_map();
    }
  
      return true;
    }

    void save_map()
    {
      std::string mapdatafile = ros::package::getPath("managment") + "/maps/map.pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] == 0) { //occ [0,0.1)
            fputc(254, out);
          } else if (map->data[i] == +100) { //occ (0.65,1]
            fputc(000, out);
          } else { //occ [0.1,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);

      std::string mapmetadatafile = mapname_ + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& _map)
    {
      if ( _map != NULL)
      {
        get_map = true;    
        ROS_INFO("Received a %d X %d map @ %.3f m/pix",_map->info.width,_map->info.height,_map->info.resolution);
        map = _map;
      }
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    bool get_map;

};

class MapServer
{
  public:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    ros::ServiceServer service2;
    bool deprecated;
    bool set_map;
    double res;
    std::string fname;
    std::string mapfname;


    MapServer(const std::string& _fname, double _res)
    {
      service2 = n.advertiseService("sepantamapengenine/load", &MapServer::checkcommand , this);
      service = n.advertiseService("sepantamapengenine/staticmap", &MapServer::mapCallback, this);
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      fname = _fname;
      res = _res;
    }

  void load_map()
    {
      double origin[3];
      int negate;
      double occ_th, free_th;
      bool trinary = true;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (true) {
        //mapfname = fname + ".pgm";
       
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }

#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        fin.close();
        try { 
          doc["resolution"] >> res; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["trinary"] >> trinary; 
        } catch (YAML::Exception) { 
          ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
          trinary = true;
        }
        try { 
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1]; 
          doc["origin"][2] >> origin[2]; 
           ROS_INFO("Origin read");
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["image"] >> mapfname; 
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("The image tag cannot be an empty string.");
            exit(-1);
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      mapfname = ros::package::getPath("managment") + "/maps/map.pgm";
      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      std::cout<<res<<std::endl;
      //////////////////////////////////////////////////////////
      map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, trinary);
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      geometry_msgs::Pose mpos;
      //EDWINTODO IF MAP CHANGED CHANGE THIS ORIGIN TO VALID VALUE , SORRY FOR HARDCODING :)
      mpos.position.x = origin[0];
      mpos.position.y = origin[1];
      std::cout<<"map origin: "<<origin[0]<<" "<<origin[1]<<std::endl;
      map_resp_.map.info.origin = mpos;
      metadata_pub.publish( meta_data_message_ );
     
      map_pub.publish( map_resp_.map );

      set_map = true;

      ROS_INFO("Read and published a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;
    }
    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,nav_msgs::GetMap::Response &res )
    {
      
        
          res = map_resp_;
          ROS_INFO("Sending map");

        
     

      return true;
    }

   bool checkcommand(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
   {
      ROS_INFO("Load Requested");
      load_map();
      return true;
    }

    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sepantamapengenine", ros::init_options::AnonymousName);
  
  ROS_INFO("sepantamapengenine Started");

  std::string fname = ros::package::getPath("managment") + "/maps/map.pgm";
  std::string fname2 = ros::package::getPath("managment") + "/maps/map.yaml";
  std::string mapname = ros::package::getPath("managment") + "/maps/map";

  try
  {
    //THE map Saver
    MapGenerator mg(mapname);
    //THE map Server
    MapServer ms(fname2,0);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("sepantamapengenine exception: %s", e.what());
    return -1;
  }

  return 0;
}

