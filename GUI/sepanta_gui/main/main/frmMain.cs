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
        public bool es = false;
        public bool isMouseDown = false;
        public Color paintColor = Color.White;
        public enum Item {Hand, Pencil}
        public Item currentTool = Item.Pencil;
        MapTool objMap = new MapTool();
        int mouseX, mouseY, mapPosX = 0, mapPosY = 0;
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
                objMap.SetMap(@"C:\Users\WPM\Documents\map.tif", img_map.Size);
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
                MessageBox.Show("There was an error opening the map.\n" +ex.Message);
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
            dialog.Filter = "TIFF Files (*.tif)|*.tif";
            dialog.InitialDirectory = @"C:\";
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
                        short pencilSize = Convert.ToInt16(pencilSize_textBox.Text);
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
                            int tempPosX = mapPosX + mouseX - e.X;
                            int tempPosY = mapPosY + mouseY - e.Y;
                            if (tempPosX <= 0 && objMap.ThumbnailMap.Width + tempPosX >= img_map.Width) mapPosX = tempPosX;
                            if (tempPosY <= 0 && objMap.ThumbnailMap.Height + tempPosY >= img_map.Height) mapPosY = tempPosY;
                            g.DrawImage(objMap.ThumbnailMap,new Point(mapPosX,mapPosY));
                        }
                        break;
                    case Item.Pencil:
                        short pencilSize = Convert.ToInt16(pencilSize_textBox.Text);
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
            paintColor = Color.Gray;
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
            dialog.Filter = "TIFF Files (*.tif)|*.tif";
            dialog.InitialDirectory = @"C:\";
            dialog.Title = "Saving map ...";
            if(dialog.ShowDialog() == DialogResult.OK)
            {
                objMap.Map.Save(dialog.FileName, ImageFormat.Tiff);
            }
        }
    }
}
