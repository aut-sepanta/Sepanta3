using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace main
{
    public partial class frmMain : Form
    {
        public frmMain()
        {
            InitializeComponent();
        }

        private void textBox6_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox5_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox4_TextChanged(object sender, EventArgs e)
        {

        }

        private void button4_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,right");
        }

        private void tabPage2_Click(object sender, EventArgs e)
        {

        }

        private void btn_set_path_Click(object sender, EventArgs e)
        {

        }

        private void frmMain_Load(object sender, EventArgs e)
        {
            statics.main_config = new config();
            statics.loadXML_config();
        }

        private void btn_connect_Click(object sender, EventArgs e)
        {
            statics.main_tcp = new client.my_client("GUI");
            statics.main_tcp.get_message += main_tcp_get_message;
            statics.main_tcp.Connect();
        }


        public void process_command(string a)
        {
            string[] array = a.Split(',');
            if ( array[0] == "UPDATE")
            {
                if (array[1] == "ODOM1")
                {
                    odom1_x.Text = array[2];
                    odom1_y.Text = array[3];
                    odom1_t.Text = array[4];
                }
            }
        }

        void main_tcp_get_message(client.ClientEventArgs args)
        {
            this.BeginInvoke(new MethodInvoker(delegate
                         {
                             lst_tcp.Items.Add(args.message);
                             process_command(args.message);
                         }));
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if ( statics.main_tcp != null)
            statics.main_tcp.write_send(txt_message.Text);
        }

        private void btn_stop_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,stop");
        }

        private void btn_up_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,forward");
        }

        private void btn_down_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,backward");
        }

        private void btn_turn_right_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,turnright");
        }

        private void btn_turn_left_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,turnleft");
        }

        private void btn_left_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,left");
        }

        private void btn_x_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,move_x," + txt_move_x.Text);
        }

        private void btn_y_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,move_y," + txt_move_y.Text);
        }

        private void turn_local_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,move_turnlocal," + txt_move_turnlocal.Text);
        }

        private void turn_to_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,move_turnto," + txt_move_turnto.Text);
        }

        private void btn_cancle_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,cancle");
        }

        private void btn_es0_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,es0");
        }

        private void btn_es1_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,es1");
        }
    }
}
