using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace main
{
    public partial class frmpoint : Form
    {
        int mode = 0;
        int selected_index = 0;
        public frmpoint()
        {
            InitializeComponent();
        }

        private void btn_ok_Click(object sender, EventArgs e)
        {
            if ( txt_x.Text == "" || txt_y.Text == "" || txt_yaw.Text == "" || txt_height.Text == "" || txt_name.Text == "")
            {
                MessageBox.Show("please fill all blanks");
                return;
            }

         

            if ( mode == 0 || mode == 3) //new and current
            {
                bool re = statics.check_dublicated(txt_name.Text);
                if (re == false) { MessageBox.Show("Duplicated name !"); return; }
                //this is a new point
                map_data d = new map_data();
                d.x = txt_x.Text;
                d.y = txt_y.Text;
                d.heigth = txt_height.Text;
                d.labe_name = txt_name.Text;
                d.yaw = txt_yaw.Text;

                statics.list_points.Add(d);
                statics.save_points();

                Close();

            }

            if ( mode == 1) //Edit
            {
                bool re = statics.check_dublicated(txt_name.Text,selected_index);
                if (re == false) { MessageBox.Show("Duplicated name !"); return; }
                statics.list_points[selected_index].x = txt_x.Text;
                statics.list_points[selected_index].y = txt_y.Text;
                statics.list_points[selected_index].yaw = txt_yaw.Text;
                statics.list_points[selected_index].heigth = txt_height.Text;
                statics.list_points[selected_index].labe_name = txt_name.Text;
                statics.save_points();

                Close();

            }
        }

        public void p_new()
        {
            mode = 0;
        }
        public void p_edit(string x,string y,string yaw,string height,string lable,int selected)
        {
            mode = 1;

            txt_x.Text = x;
            txt_y.Text = y;
            txt_yaw.Text = yaw;
            txt_height.Text = height;
            txt_name.Text = lable;
            selected_index = selected;
           
        }
        public void p_delete()
        {

        }

        public void p_current(string x,string y,string yaw,string height,string lable)
        {
            mode = 3;
            txt_x.Text = x;
            txt_y.Text = y;
            txt_yaw.Text = yaw;
            txt_height.Text = height;
            txt_name.Text = lable;
        }
        private void frmpoint_Load(object sender, EventArgs e)
        {

        }
    }
}
