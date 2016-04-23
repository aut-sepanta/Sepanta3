using Android.Content;
using System;
using System.IO;
using System.Xml.Serialization;
namespace sepanta
{
    [XmlRootAttribute("config")]
    public class config
    {
        public string remote_ip = "192.168.1.4";
        public int remote_port = 3000;
    }

    public class statics
    {
        public static bool es_mode = false;
        public static config main_config;
      
        public static string sd_configpath = "/sdcard/Sepanta/config.xml";
        public static string sd_configpath_folder = "/sdcard/Sepanta/";

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

        public static object  XMLload_data(string path, Type type)
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
            statics.XMLsave_data(statics.sd_configpath, typeof(config), statics.main_config);
        }

        public static void loadXML_config()
        {
            bool r2 = System.IO.Directory.Exists(sd_configpath_folder);
            if (r2 == false)
            {
                System.IO.Directory.CreateDirectory(sd_configpath_folder);
            }
            bool r = System.IO.File.Exists(sd_configpath);
            if (r == false) { saveXML_config(); return; }

            try
            {
                statics.main_config = (config)statics.XMLload_data(statics.sd_configpath, typeof(config));
            }
            catch ( Exception e)
            {
                saveXML_config();
            }
        }

      

        #region region_show
        public static void show_error(string msg, Context cx)
        {
            AndroidHUD.AndHUD.Shared.ShowError(cx, msg, AndroidHUD.MaskType.Clear, new TimeSpan(0, 0, 2), null, null);
        }

        public static void show_success(string msg, Context cx)
        {
            AndroidHUD.AndHUD.Shared.ShowSuccess(cx, msg, AndroidHUD.MaskType.Clear, new TimeSpan(0, 0, 2), null, null);
        }

        public static void show_waiting(string msg, Context cx)
        {
            AndroidHUD.AndHUD.Shared.Show(cx, msg, -1, AndroidHUD.MaskType.Clear);
        }
        public static void hide_watting(Context cx)
        {
            AndroidHUD.AndHUD.Shared.Dismiss(cx);
        }
        #endregion 
    }
}
