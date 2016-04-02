using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Xml.Serialization;

namespace main
{
    public class config
    {
        List<map_data> list_points = new List<map_data>();
    }

    public class map_data
    {
        string labe_name;
        float x;
        float y;
        float z;
        float teta;
    }

    class statics
    {
        public static frmMain main_rec;
        public static client.my_client main_tcp;
        public static config main_config;

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
            statics.XMLsave_data(Environment.CurrentDirectory + "/config.xml", typeof(config), statics.main_config);
        }

        public static void loadXML_config()
        {
            //bool r2 = System.IO.Directory.Exists(Environment.CurrentDirectory);
            //if (r2 == false)
            //{
            //    System.IO.Directory.CreateDirectory(sd_configpath_folder);
            //}
            bool r = System.IO.File.Exists(Environment.CurrentDirectory + "/config.xml");
            if (r == false) { saveXML_config(); return; }

            try
            {
                statics.main_config = (config)statics.XMLload_data(Environment.CurrentDirectory, typeof(config));
            }
            catch (Exception ex)
            {
                saveXML_config();
            }
        }

    }
}
