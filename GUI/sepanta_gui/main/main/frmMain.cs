using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Text;
using System.Windows.Forms;

namespace main
{
    public partial class frmMain : Form
    {
        public const int max_manager_nodes = 20;
        public bool es = false;
        public bool isMouseDown = false;
        public Color paintColor = Color.White;
        public enum Item {Hand, Pencil}
        public Item currentTool = Item.Pencil;
        MapTool objMap = new MapTool();
        int mouseX, mouseY, mapPosX = 0, mapPosY = 0;

        List<Button> manager_btn_list = new List<Button>();

       class manager_node
       {
           public string status;
         
       }

        List<manager_node> manager_node_list = new List<manager_node>();

        public frmMain()
        {
            InitializeComponent();

            for (int i = 0 ; i < max_manager_nodes ; i++)
            {
                manager_node n = new manager_node();
                n.status = "stop";

                manager_node_list.Add(n);
            }

            manager_btn_list.Add(btn_m1);
            manager_btn_list.Add(btn_m2);
            manager_btn_list.Add(btn_m3);
            manager_btn_list.Add(btn_m4);
            manager_btn_list.Add(btn_m5);
            manager_btn_list.Add(btn_m6);
            manager_btn_list.Add(btn_m7);
            manager_btn_list.Add(btn_m8);
            manager_btn_list.Add(btn_m9);
            manager_btn_list.Add(btn_m10);
            manager_btn_list.Add(btn_m11);
            manager_btn_list.Add(btn_m12);
            manager_btn_list.Add(btn_m13);
            manager_btn_list.Add(btn_m14);
            manager_btn_list.Add(btn_m15);
            manager_btn_list.Add(btn_m16);
            manager_btn_list.Add(btn_m17);
            manager_btn_list.Add(btn_m18);
            manager_btn_list.Add(btn_m19);
            manager_btn_list.Add(btn_m20);

            update_manager_page();
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

        void load_points()
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
            statics.init();
            statics.load_points();

            txt_ip.Text = statics.main_config.ip;
            txt_port.Text = statics.main_config.port.ToString();

            update_list_point();
            txt_map.Text = track_size.Value.ToString();
            txt_path.Text = statics.main_config.linux_path;

            if (statics.main_config.mode == 0)
                rad_linux.Checked = true;
            else
                rad_windows.Checked = true;
        }

        private void btn_connect_Click(object sender, EventArgs e)
        {
            try
            {
                statics.main_tcp = new client.my_client("GUI");
                statics.main_config.ip = txt_ip.Text;
                statics.main_config.port = int.Parse(txt_port.Text);
                statics.main_tcp.ip = statics.main_config.ip;
                statics.main_tcp.port = statics.main_config.port;
                statics.main_tcp.get_message += main_tcp_get_message;
                statics.main_tcp.Connect();
                statics.saveXML_config();
            }
            catch
            {

            }
        }

        public void update_manager_page()
        {
            for ( int i = 0 ; i < manager_node_list.Count ; i++)
            {
                if (manager_node_list[i].status == "0")
                    manager_btn_list[i].BackColor = Color.DarkRed;
                if ( manager_node_list[i].status == "2")
                     manager_btn_list[i].BackColor = Color.Green;
                if (manager_node_list[i].status == "1")
                        manager_btn_list[i].BackColor = Color.DarkOrange;
            }
          
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

                if (array[1] == "ODOM2")
                {
                    odom2_x.Text = array[2];
                    odom2_y.Text = array[3];
                    odom2_t.Text = array[4];
                }
            }
            if ( array[0] == "MANAGER")
            {
                for ( int i = 0 ; i < max_manager_nodes ; i++)
                {
                   manager_node_list[i].status = array[1 + i];
                }

                update_manager_page();
             }
        }

        void main_tcp_get_message(client.ClientEventArgs args)
        {
            try
            {
                this.BeginInvoke(new MethodInvoker(delegate
                             {
                                 if (!args.message.Contains("MANAGER"))
                                 {
                                     lst_tcp.Items.Add(args.message);
                                 }
                                 process_command(args.message);
                             }));
            }
            catch
            {

            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if ( statics.main_tcp != null)
            statics.main_tcp.write_send(txt_ip.Text);
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

        private void btn_es_Click(object sender, EventArgs e)
        {
            if (es)
            {
                statics.global_send("COMMAND,es0");
                es = false;
                btn_es.BackColor = Color.White;
            }
            else
            {
                statics.global_send("COMMAND,es1");
                es = true;
                btn_es.BackColor = Color.Red;
            }
        }

        private void btn_get_current_map_Click(object sender, EventArgs e)
        {
            try
            {
                objMap.SetMap(statics._path +  "map.pgm", img_map.Size);
                img_map.Image = objMap.ThumbnailMap;
                btn_save_map.Enabled = true;
                btn_add_current.Enabled = true;
                btn_edit_selected.Enabled = true;
                btn_delete_selected.Enabled = true;
                btn_show_lables.Enabled = true;
                btn_zoom_in.Enabled = true;
                btn_zoom_out.Enabled = true;
                mapPosX = 0;
                mapPosY = 0;
            }
            catch(Exception ex)
            {
                MessageBox.Show("There was an error opening the map.\n" + ex.Message);
            }
        }

        private void btn_zoom_in_Click(object sender, EventArgs e)
        {
            //float tempScale = objMap.Scale;
            objMap.ZoomIn();
            //mapPosX = (int)(mapPosX * (objMap.Scale / tempScale));
            //mapPosY = (int)(mapPosY * (objMap.Scale / tempScale));
            mapPosX = 0;
            mapPosY = 0;
            img_map.Image = objMap.ThumbnailMap;
            //Graphics g = img_map.CreateGraphics();
            //g.DrawImage(objMap.ThumbnailMap, new Point(mapPosX, mapPosY));
            //g.Dispose();
        }

        private void btn_zoom_out_Click(object sender, EventArgs e)
        {
            //float tempScale = objMap.Scale;
            objMap.ZoomOut();
            //mapPosX = (int)(mapPosX * (objMap.Scale / tempScale));
            //mapPosY = (int)(mapPosY * (objMap.Scale / tempScale));
            mapPosX = 0;
            mapPosY = 0;
            img_map.Image = objMap.ThumbnailMap;
            //Graphics g = img_map.CreateGraphics();
            //g.DrawImage(objMap.ThumbnailMap, new Point(mapPosX, mapPosY));
            //g.Dispose();
        }

        private void btn_load_map_Click(object sender, EventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            dialog.Filter = "PGM Files (*.pgm)|*.pgm";
            dialog.InitialDirectory = statics._path;
            dialog.Title = "Please select map";
            if (dialog.ShowDialog() == DialogResult.OK)
            {
                objMap.SetMap(dialog.FileName, img_map.Size);
                img_map.Image = objMap.ThumbnailMap;
                btn_save_map.Enabled = true;
                btn_add_current.Enabled = true;
                btn_edit_selected.Enabled = true;
                btn_delete_selected.Enabled = true;
                btn_show_lables.Enabled = true;
                btn_zoom_in.Enabled = true;
                btn_zoom_out.Enabled = true;
                mapPosX = 0;
                mapPosY = 0;
            }
        }

        private void img_map_MouseDown(object sender, MouseEventArgs e)
        {
            if (objMap.IsMapInit)
            {
                isMouseDown = true;
                mouseX = e.X;
                mouseY = e.Y;
                switch (currentTool)
                {
                    case Item.Hand:
                        break;
                    case Item.Pencil:
                        short pencilSize = Convert.ToInt16(track_size.Value.ToString());
                        Graphics g = Graphics.FromImage(objMap.Map);
                        g.FillRectangle(new SolidBrush(paintColor), ((e.X - mapPosX) / objMap.Scale) - pencilSize / 2, ((e.Y - mapPosY) / objMap.Scale) - pencilSize / 2, pencilSize, pencilSize);
                        g = img_map.CreateGraphics();
                        g.DrawImage(objMap.ThumbnailMap,new Point(mapPosX,mapPosY));
                        g.Dispose();
                        break;
                    default:
                        break;
                }
            }
        }

        private void img_map_MouseUp(object sender, MouseEventArgs e)
        {
            isMouseDown = false;
        }

        private void img_map_MouseMove(object sender, MouseEventArgs e)
        {
            if (isMouseDown)
            {
                Graphics g = img_map.CreateGraphics();
                switch (currentTool)
                {
                    case Item.Hand:
                        if(img_map.Image.Width > img_map.Width || img_map.Image.Height > img_map.Height)
                        {
                            int tempPosX = mapPosX - mouseX + e.X;
                            int tempPosY = mapPosY - mouseY + e.Y;
                            if (tempPosX <= 0 && objMap.ThumbnailMap.Width + tempPosX >= img_map.Width) mapPosX = tempPosX;
                            if (tempPosY <= 0 && objMap.ThumbnailMap.Height + tempPosY >= img_map.Height) mapPosY = tempPosY;
                            g.DrawImage(objMap.ThumbnailMap,new Point(mapPosX,mapPosY));
                        }
                        break;
                    case Item.Pencil:
                        short pencilSize = Convert.ToInt16(track_size.Value.ToString());
                        g = Graphics.FromImage(objMap.Map);
                        g.FillRectangle(new SolidBrush(paintColor), ((e.X - mapPosX) / objMap.Scale) - pencilSize / 2, ((e.Y - mapPosY) / objMap.Scale) - pencilSize / 2, pencilSize, pencilSize);
                        g = img_map.CreateGraphics();
                        g.DrawImage(objMap.ThumbnailMap,new Point(mapPosX,mapPosY));
                        break;
                    default:
                        break;
                }
                g.Dispose();
            }
        }

        private void btn_paint_w_Click(object sender, EventArgs e)
        {
            paintColor = Color.White;
            btn_paint_w.Enabled = false;
            btn_paint_b.Enabled = true;
            btn_paint_g.Enabled = true;
        }

        private void btn_paint_b_Click(object sender, EventArgs e)
        {
            paintColor = Color.Black;
            btn_paint_w.Enabled = true;
            btn_paint_b.Enabled = false;
            btn_paint_g.Enabled = true;
        }

        private void btn_paint_g_Click(object sender, EventArgs e)
        {
            paintColor = Color.FromArgb(205, 205, 205);
            btn_paint_w.Enabled = true;
            btn_paint_b.Enabled = true;
            btn_paint_g.Enabled = false;
        }

        private void btn_selectHand_Click(object sender, EventArgs e)
        {
            btn_selectHand.Enabled = false;
            btn_selectPencil.Enabled = true;
            currentTool = Item.Hand;
        }

        private void btn_selectPencil_Click(object sender, EventArgs e)
        {
            btn_selectHand.Enabled = true;
            btn_selectPencil.Enabled = false;
            currentTool = Item.Pencil;
        }

        private void btn_save_map_Click(object sender, EventArgs e)
        {
            SaveFileDialog dialog = new SaveFileDialog();
            dialog.Filter = "PGM Files (*.pgm)|*.pgm";
            dialog.InitialDirectory = statics._path;
            dialog.Title = "Saving map ...";
            if(dialog.ShowDialog() == DialogResult.OK)
            {
                objMap.SaveMap(dialog.FileName);
            }
        }

        private void label3_Click(object sender, EventArgs e)
        {

        }

        private void tabPage4_Click(object sender, EventArgs e)
        {

        }

    
        public void update_list_point()
        {
            lst_points.Items.Clear();
            for ( int i = 0 ; i < statics.list_points.Count ; i++)
            {
                lst_points.Items.Add(statics.list_points[i].labe_name);
            }
            
        }

        private void btn_edit_selected_Click(object sender, EventArgs e)
        {
            int index = lst_points.SelectedIndex;
            if (index == -1) return;
            map_data d = statics.list_points[index];

            frmpoint p = new frmpoint();
            p.p_edit(d.x, d.y, d.yaw, d.heigth, d.labe_name,index);
            
            p.ShowDialog();
        }

        private void btn_add_current_Click(object sender, EventArgs e)
        {
            frmpoint p = new frmpoint();
            p.p_current(statics.odom_hector.x, statics.odom_hector.y, statics.odom_hector.yaw, "100", "");
            p.ShowDialog();
            update_list_point();
        }

        private void button4_Click_1(object sender, EventArgs e)
        {
            frmpoint p = new frmpoint();
            p.ShowDialog();
            update_list_point();
        }

        private void btn_delete_selected_Click(object sender, EventArgs e)
        {
            int index = lst_points.SelectedIndex;
            if (index == -1) return;

            DialogResult dialogResult = MessageBox.Show("Sure ?", "Sepanta GUI", MessageBoxButtons.YesNo);
            if (dialogResult == DialogResult.Yes)
            {
                //do something
                statics.list_points.RemoveAt(index);
                update_list_point();
                statics.save_points();
          
            }
            else if (dialogResult == DialogResult.No)
            {
                //do something else
            }


        }

        private void btn_disconnect_Click(object sender, EventArgs e)
        {
            if (statics.main_tcp != null)
            {
                statics.main_tcp.Disconnect();
                statics.main_tcp = null;
                chk_coonect.Checked = false;
            }
            
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (statics.main_tcp == null ||  statics.main_tcp.active == false)
            {
                if ( chk_coonect.Checked )
                btn_connect_Click(null, null);
            }
        }

        private void frmMain_FormClosed(object sender, FormClosedEventArgs e)
        {
            if ( statics.main_tcp != null)
            {
                statics.main_tcp.Disconnect();
                statics.main_tcp = null;
            }
        }

        private void lst_points_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            btn_edit_selected_Click(null, null);
        }

        private void track_size_Scroll(object sender, EventArgs e)
        {
            txt_map.Text = track_size.Value.ToString();
        }

        private void rad_windows_CheckedChanged(object sender, EventArgs e)
        {
            statics.main_config.mode = 1;
            statics.saveXML_config();
            statics.init();

            txt_path.Text = statics.main_config.test_path;
        }

        private void rad_linux_CheckedChanged(object sender, EventArgs e)
        {
            statics.main_config.mode = 0;
            statics.saveXML_config();
            statics.init();

            txt_path.Text = statics.main_config.linux_path;
        }

        private void btn_hector_offset_Click(object sender, EventArgs e)
        {
            statics.global_send("COMMAND,offset_hector," + txt_offsetx.Text + "," + txt_offsety.Text);
            txt_offsetx.Text = "";
            txt_offsety.Text = "";
        }

        private void btn_m1_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,1");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,1");
        }

        private void btn_m2_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,2");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,2");
        }

        private void btn_m3_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,3");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,3");
        }

        private void btn_m4_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,4");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,4");
        }

        private void btn_m5_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,5");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,5");
        }

        private void btn_m6_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,6");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,6");
        }

        private void btn_m7_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,7");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,7");
        }

        private void btn_m8_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,8");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,8");
        }

        private void btn_m9_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,9");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,9");
        }

        private void btn_m10_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,10");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,10");
        }

        private void btn_m11_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,11");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,11");
        }

        private void btn_m12_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,12");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,12");
        }

        private void btn_m13_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,13");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,13");
        }

        private void btn_m14_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,14");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,14");
        }

        private void btn_m15_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,15");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,15");
        }

        private void btn_m16_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,16");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,16");
        }

        private void btn_m17_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,17");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,17");
        }

        private void btn_m18_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,18");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,18");
        }

        private void btn_m19_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,19");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,19");
        }

        private void btn_m20_Click(object sender, EventArgs e)
        {
            if (rad_start.Checked)
                statics.global_send("MANAGER,start,20");
            if (rad_stop.Checked)
                statics.global_send("MANAGER,stop,20");
        }

      

     
    }
}
