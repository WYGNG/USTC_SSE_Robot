namespace 双目测试
{
    partial class Form1
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.label1 = new System.Windows.Forms.Label();
            this.cmbCamera = new System.Windows.Forms.ComboBox();
            this.label2 = new System.Windows.Forms.Label();
            this.cmbResolution = new System.Windows.Forms.ComboBox();
            this.vispShoot = new AForge.Controls.VideoSourcePlayer();
            this.picbPreview = new System.Windows.Forms.PictureBox();
            this.btnConnect = new System.Windows.Forms.Button();
            this.btnDisconnect = new System.Windows.Forms.Button();
            this.btnShoot = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)(this.picbPreview)).BeginInit();
            this.SuspendLayout();
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(39, 22);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(53, 12);
            this.label1.TabIndex = 0;
            this.label1.Text = "摄像头：";
            // 
            // cmbCamera
            // 
            this.cmbCamera.FormattingEnabled = true;
            this.cmbCamera.Location = new System.Drawing.Point(98, 19);
            this.cmbCamera.Name = "cmbCamera";
            this.cmbCamera.Size = new System.Drawing.Size(113, 20);
            this.cmbCamera.TabIndex = 1;
            this.cmbCamera.SelectedIndexChanged += new System.EventHandler(this.cmbCamera_SelectedIndexChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(217, 22);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 12);
            this.label2.TabIndex = 0;
            this.label2.Text = "分辨率：";
            // 
            // cmbResolution
            // 
            this.cmbResolution.FormattingEnabled = true;
            this.cmbResolution.Location = new System.Drawing.Point(276, 19);
            this.cmbResolution.Name = "cmbResolution";
            this.cmbResolution.Size = new System.Drawing.Size(126, 20);
            this.cmbResolution.TabIndex = 1;
            // 
            // vispShoot
            // 
            this.vispShoot.Location = new System.Drawing.Point(52, 54);
            this.vispShoot.Name = "vispShoot";
            this.vispShoot.Size = new System.Drawing.Size(833, 332);
            this.vispShoot.TabIndex = 2;
            this.vispShoot.Text = "videoSourcePlayer1";
            this.vispShoot.VideoSource = null;
            // 
            // picbPreview
            // 
            this.picbPreview.Location = new System.Drawing.Point(52, 410);
            this.picbPreview.Name = "picbPreview";
            this.picbPreview.Size = new System.Drawing.Size(833, 346);
            this.picbPreview.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
            this.picbPreview.TabIndex = 3;
            this.picbPreview.TabStop = false;
            // 
            // btnConnect
            // 
            this.btnConnect.Location = new System.Drawing.Point(408, 22);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(80, 23);
            this.btnConnect.TabIndex = 4;
            this.btnConnect.Text = "连接";
            this.btnConnect.UseVisualStyleBackColor = true;
            this.btnConnect.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // btnDisconnect
            // 
            this.btnDisconnect.Enabled = false;
            this.btnDisconnect.Location = new System.Drawing.Point(494, 22);
            this.btnDisconnect.Name = "btnDisconnect";
            this.btnDisconnect.Size = new System.Drawing.Size(80, 23);
            this.btnDisconnect.TabIndex = 4;
            this.btnDisconnect.Text = "断开";
            this.btnDisconnect.UseVisualStyleBackColor = true;
            this.btnDisconnect.Click += new System.EventHandler(this.btnDisconnect_Click);
            // 
            // btnShoot
            // 
            this.btnShoot.Enabled = false;
            this.btnShoot.Location = new System.Drawing.Point(578, 22);
            this.btnShoot.Name = "btnShoot";
            this.btnShoot.Size = new System.Drawing.Size(80, 23);
            this.btnShoot.TabIndex = 4;
            this.btnShoot.Text = "拍照";
            this.btnShoot.UseVisualStyleBackColor = true;
            this.btnShoot.Click += new System.EventHandler(this.btnShoot_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(8, 64);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(41, 12);
            this.label3.TabIndex = 5;
            this.label3.Text = "视频：";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(5, 421);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(41, 12);
            this.label4.TabIndex = 6;
            this.label4.Text = "拍照：";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(918, 792);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.btnShoot);
            this.Controls.Add(this.btnDisconnect);
            this.Controls.Add(this.btnConnect);
            this.Controls.Add(this.picbPreview);
            this.Controls.Add(this.vispShoot);
            this.Controls.Add(this.cmbResolution);
            this.Controls.Add(this.cmbCamera);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Name = "Form1";
            this.Text = "Form1";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.Load += new System.EventHandler(this.Form1_Load);
            ((System.ComponentModel.ISupportInitialize)(this.picbPreview)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox cmbCamera;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.ComboBox cmbResolution;
        private AForge.Controls.VideoSourcePlayer vispShoot;
        private System.Windows.Forms.PictureBox picbPreview;
        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.Button btnDisconnect;
        private System.Windows.Forms.Button btnShoot;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
    }
}