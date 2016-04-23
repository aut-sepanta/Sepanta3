using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Xml.Serialization;

namespace main
{
    public class config
    {
       
        public string linux_path = "~/catkin_ws/src/Sepanta3/Managment/maps/";
        public string test_path = "C:\\maps\\";
        public string ip = "127.0.0.1";
        public int port = 3000;
        public int mode = 0; //0 windows //1 linux
    }

    public class map_data
    {
        public string labe_name;
        public string x;
        public string y;
        public string heigth;
        public string yaw;
    }

    struct odom_data
    {
        public string x;
        public string y;
        public string yaw;
    }
    class statics
    {
        public static frmMain main_rec;
        public static client.my_client main_tcp;
        public static config main_config;
        public static List<map_data> list_points = new List<map_data>();
        public static string _path;
       

        public static odom_data odom_base;
        public static odom_data odom_hector;

        public static bool check_dublicated(string name)
        {
            for ( int i = 0 ; i < statics.list_points.Count ; i++)
            {
                if ( statics.list_points[i].labe_name == name )
                {
                    return false;
                }
            }
            return true;
        }
        public static bool check_dublicated(string name,int index_ignore)
        {
            for (int i = 0; i < statics.list_points.Count; i++)
            {
                if (statics.list_points[i].labe_name == name && i != index_ignore)
                {
                   
                    return false;
                }
            }
            return true;
        }
        public static void init()
        {
            if ( statics.main_config.mode == 1 )
            {
                _path = statics.main_config.linux_path;
            }
            else
            {
                _path = statics.main_config.test_path;
            }

           
        }

        public static void load_points()
        {
            if ( Directory.Exists(_path ) == false)
            {
                Directory.CreateDirectory(_path );
            }
            FileStream fs = new FileStream(_path + "points.txt", FileMode.OpenOrCreate,FileAccess.ReadWrite );
            StreamReader sr = new StreamReader(fs);
            string line = sr.ReadLine();
            while (line != "" && line != null)
            {
                string[] array = line.Split(',');
                map_data d = new map_data();
                d.labe_name = array[0];
                d.x = array[1];
                d.y = array[2];
                d.yaw = array[3];
                d.heigth = array[4];
                statics.list_points.Add(d);
                line = sr.ReadLine();
            }
            sr.Close();
            fs.Close();
        }

        public static void save_points()
        {
            if (Directory.Exists(_path ) == false)
            {
                Directory.CreateDirectory(_path );
            }
            FileStream fs = new FileStream(_path + "points.txt", FileMode.Create, FileAccess.ReadWrite);
            StreamWriter sw = new StreamWriter(fs);

            for (int i = 0; i < statics.list_points.Count; i++ )
            {
                map_data d = statics.list_points[i];
                sw.WriteLine(d.labe_name + "," + d.x + "," + d.y + "," + d.yaw + "," + d.heigth);
            }

            sw.Close();
            fs.Close();
        }

     

        public static void global_send(string message)
        {
            if ( main_tcp != null && main_tcp.active)
            {
                main_tcp.write_send("%" + message + "$");
            }
        }
        public static void XMLsave_data(string path, Type type, object obj)
        {
            try
            {
                XmlSerializer serializer = new XmlSerializer(type);
                TextWriter writer = new StreamWriter(path);
                serializer.Serialize(writer, obj);
                writer.Close();
            }
            catch
            {
            }
        }

        public static object XMLload_data(string path, Type type)
        {
            object output = null;

            XmlSerializer serializer = new XmlSerializer(type);
            FileStream reader = new FileStream(path, FileMode.Open);
            output = serializer.Deserialize(reader);
            reader.Close();

            return output;
        }

        public static void saveXML_config()
        {
            statics.XMLsave_data(Environment.CurrentDirectory + "\\config.xml", typeof(config), statics.main_config);
        }

        public static void loadXML_config()
        {      
            bool r = System.IO.File.Exists(Environment.CurrentDirectory + "\\config.xml");
            if (r == false) { saveXML_config(); return; }

            try
            {
                statics.main_config = (config)statics.XMLload_data(Environment.CurrentDirectory + "\\config.xml", typeof(config));
            }
            catch (Exception ex)
            {
                saveXML_config();
            }
        }

    }
}
