using System;
using Android.App;
using Android.Content;
using Android.Runtime;
using Android.Views;
using Android.Widget;
using Android.OS;
using client;

namespace sepanta
{
    [Activity(Label = "Sepanta", MainLauncher = true, Icon = "@drawable/logo_small")]
    public class MainActivity : Activity
    {
        int count = 1;
        my_client client;

        Button btn_forwad;
        Button btn_backward;
        Button btn_left;
        Button btn_right;
        Button btn_tleft;
        Button btn_tright;
        Button btn_es;
        Button btn_f;
        Button btn_stop;

        Button btn_connect;
        Button btn_disconnect;

        protected override void OnCreate(Bundle bundle)
        {
            base.OnCreate(bundle);
            SetContentView(Resource.Layout.Main);

            statics.main_config = new config();
            statics.loadXML_config();

            btn_forwad = FindViewById<Button>(Resource.Id.btn_forward);
            btn_backward = FindViewById<Button>(Resource.Id.btn_backward);
            btn_stop = FindViewById<Button>(Resource.Id.btn_stop);
            btn_left = FindViewById<Button>(Resource.Id.btn_left);
            btn_right = FindViewById<Button>(Resource.Id.btn_right);
            btn_tleft = FindViewById<Button>(Resource.Id.btn_tleft);
            btn_tright = FindViewById<Button>(Resource.Id.btn_tright);
            btn_es = FindViewById<Button>(Resource.Id.btn_es);
            btn_f = FindViewById<Button>(Resource.Id.btn_function);

            btn_connect = FindViewById<Button>(Resource.Id.btn_connect);
            btn_disconnect = FindViewById<Button>(Resource.Id.btn_disconnect);

            btn_forwad.Click += btn_forwad_Click;
            btn_backward.Click += btn_backward_Click;
            btn_stop.Click += btn_stop_Click;
            btn_tleft.Click += btn_tleft_Click;
            btn_tright.Click += btn_tright_Click;
            btn_f.Click += btn_f_Click;
            btn_es.Click += btn_es_Click;
            btn_left.Click += btn_left_Click;
            btn_right.Click += btn_right_Click;

            btn_connect.Click += btn_connect_Click;
            btn_disconnect.Click += btn_disconnect_Click;


            btn_es.SetBackgroundColor(Android.Graphics.Color.Green);
        }

        void btn_disconnect_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.Disconnect();
            client = null;
        }

        void btn_connect_Click(object sender, EventArgs e)
        {
            client = new my_client("android");
            statics.loadXML_config();
            client.ip = statics.main_config.remote_ip;
            client.port = statics.main_config.remote_port;
            client.get_message += client_get_message;
            //client.get_event += client_get_event;
            bool resut = client.Connect();

            if (resut)
            {
                statics.show_success("Well done :)", this);
            }
            else
            {
                statics.show_error("Somthing went wrong :(", this);
            }
        }

        void client_get_event(ClientEventArgs args)
        {
            
        }

        void client_get_message(ClientEventArgs args)
        {
            
        }

        void btn_right_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,right");
        }

        void btn_left_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,left");
        }

        void btn_es_Click(object sender, EventArgs e)
        {
            if (client == null) return;

           

            statics.es_mode = !statics.es_mode;

            if ( statics.es_mode )
            {
                client.write_send("COMMAND,es1");
                btn_es.SetBackgroundColor(Android.Graphics.Color.Red);
            }
            else
            {
                client.write_send("COMMAND,es0");
                btn_es.SetBackgroundColor(Android.Graphics.Color.Green);
            }
            
        }

        void btn_f_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,function");
        }

        void btn_tright_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,turnright");
        }

        void btn_tleft_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,turnleft");
        }

        void btn_stop_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,stop");
        }

        void btn_backward_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,backward");
        }

        void btn_forwad_Click(object sender, EventArgs e)
        {
            if (client == null) return;
            client.write_send("COMMAND,forward");
        }
    }
}

