namespace main
{
    partial class frmpoint
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.txt_y = new System.Windows.Forms.TextBox();
            this.txt_yaw = new System.Windows.Forms.TextBox();
            this.txt_height = new System.Windows.Forms.TextBox();
            this.txt_name = new System.Windows.Forms.TextBox();
            this.txt_x = new System.Windows.Forms.TextBox();
            this.btn_ok = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // txt_y
            // 
            this.txt_y.Location = new System.Drawing.Point(12, 61);
            this.txt_y.Name = "txt_y";
            this.txt_y.Size = new System.Drawing.Size(164, 20);
            this.txt_y.TabIndex = 0;
            this.txt_y.Text = "0";
            // 
            // txt_yaw
            // 
            this.txt_yaw.Location = new System.Drawing.Point(12, 99);
            this.txt_yaw.Name = "txt_yaw";
            this.txt_yaw.Size = new System.Drawing.Size(164, 20);
            this.txt_yaw.TabIndex = 1;
            this.txt_yaw.Text = "0";
            // 
            // txt_height
            // 
            this.txt_height.Location = new System.Drawing.Point(12, 138);
            this.txt_height.Name = "txt_height";
            this.txt_height.Size = new System.Drawing.Size(164, 20);
            this.txt_height.TabIndex = 2;
            this.txt_height.Text = "0";
            // 
            // txt_name
            // 
            this.txt_name.Location = new System.Drawing.Point(12, 176);
            this.txt_name.Name = "txt_name";
            this.txt_name.Size = new System.Drawing.Size(164, 20);
            this.txt_name.TabIndex = 3;
            this.txt_name.Text = "none";
            // 
            // txt_x
            // 
            this.txt_x.Location = new System.Drawing.Point(12, 25);
            this.txt_x.Name = "txt_x";
            this.txt_x.Size = new System.Drawing.Size(164, 20);
            this.txt_x.TabIndex = 4;
            this.txt_x.Text = "0";
            // 
            // btn_ok
            // 
            this.btn_ok.Location = new System.Drawing.Point(12, 204);
            this.btn_ok.Name = "btn_ok";
            this.btn_ok.Size = new System.Drawing.Size(164, 23);
            this.btn_ok.TabIndex = 5;
            this.btn_ok.Text = "Ok";
            this.btn_ok.UseVisualStyleBackColor = true;
            this.btn_ok.Click += new System.EventHandler(this.btn_ok_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(13, 6);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(26, 13);
            this.label1.TabIndex = 6;
            this.label1.Text = "(X) :";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(13, 45);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(26, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "(Y) :";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(13, 83);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(40, 13);
            this.label3.TabIndex = 8;
            this.label3.Text = "(Yaw) :";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(13, 122);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(50, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "(Height) :";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(13, 160);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(47, 13);
            this.label5.TabIndex = 10;
            this.label5.Text = "(Name) :";
            // 
            // frmpoint
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(188, 239);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.btn_ok);
            this.Controls.Add(this.txt_x);
            this.Controls.Add(this.txt_name);
            this.Controls.Add(this.txt_height);
            this.Controls.Add(this.txt_yaw);
            this.Controls.Add(this.txt_y);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedToolWindow;
            this.Name = "frmpoint";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Point";
            this.Load += new System.EventHandler(this.frmpoint_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox txt_y;
        private System.Windows.Forms.TextBox txt_yaw;
        private System.Windows.Forms.TextBox txt_height;
        private System.Windows.Forms.TextBox txt_name;
        private System.Windows.Forms.TextBox txt_x;
        private System.Windows.Forms.Button btn_ok;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
    }
}