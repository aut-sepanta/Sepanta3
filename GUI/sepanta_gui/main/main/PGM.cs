using System;
using System.Text;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;

namespace main
{
    public class PGM
    {
        private int mWidth;
        private int mLength; private int mColor;
        private string mType;
        private byte[] mData; private string mComments;

        public string Comment
        {
            get { return this.mComments; }
        }

        public int Width
        {
            get { return this.mWidth; }
        }

        public int Length
        {
            get { return this.mLength; }
        }

        public int ColorSize
        {
            get { return this.mColor; }
        }
        public string Type
        {
            get
            {
                return this.mType;
            }
        }

        public string Header
        {
            get
            {
                return this.Type + Convert.ToChar(10) +
                       '#' + this.Comment + Convert.ToChar(10) +
                        this.mWidth.ToString() + " " + this.mLength.ToString() + Convert.ToChar(10) +
                        this.mColor.ToString() + Convert.ToChar(10);
            }
        }

        public byte[] Data
        {
            get { return this.mData; }
            set { this.mData = value; }
        }


        public PGM(string _filePath)
        {
            ReadPGM(_filePath);
        }

        public Bitmap ToBitmap()
        {
            Bitmap returnBitmap = new Bitmap(Width, Length);
            Graphics g = Graphics.FromImage(returnBitmap);
            for (int Xcount = 0; Xcount < Width; Xcount++)
            {
                for (int Ycount = 0; Ycount < Length; Ycount++)
                {
                    int colorValue = Data[Ycount * Length + Xcount];
                    returnBitmap.SetPixel(Xcount, Ycount, Color.FromArgb(colorValue, colorValue, colorValue));
                }
            }
            g.Dispose();
            return returnBitmap;
        }

        public void Save(string _filePath) 
        { 
            WritePGM(_filePath); 
        }
        private void ReadPGM(string _filePath)
        {
            FileStream InputStream = File.OpenRead(_filePath);
            BinaryReader PGMReader = new BinaryReader(InputStream);
            char[] Seperators = { ' ', '\n' };
            byte NewLineAsciiCode = 10;
            byte DiezAsciiCode = 35;
            byte SpaceAsciiCode = 32;
            byte[] TempArray = new byte[1000];
            int i = 0;

            string TempS;
            byte TempByte;
            /* Sample PGM :
             * 
             * 
             * P2
             * # Created by ...
             * 512 512
             * 255
             * [data]
             */
            //read PGM Type P2, P5
            TempArray[0] = PGMReader.ReadByte();
            TempArray[1] = PGMReader.ReadByte();
            this.mType = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, 2);

            //read until new line
            while (PGMReader.ReadByte() != NewLineAsciiCode) { ;}

            //read comments if exists. Only one comment line supported!!
            i = 0;
            TempArray[i] = PGMReader.ReadByte();

            if (TempArray[i] == DiezAsciiCode)
            {
                TempByte = PGMReader.ReadByte();
                while (TempByte != NewLineAsciiCode)
                {
                    TempArray[i++] = TempByte;
                    TempByte = PGMReader.ReadByte();
                }
            }
            this.mComments = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, i);

            //read width
            i = 0;
            TempByte = PGMReader.ReadByte();
            while (TempByte != SpaceAsciiCode)
            {
                TempArray[i++] = TempByte;
                TempByte = PGMReader.ReadByte();
            }

            TempS = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, i);
            this.mWidth = Convert.ToInt32(TempS);

            //read length
            i = 0;
            TempByte = PGMReader.ReadByte();
            while (TempByte != NewLineAsciiCode)
            {
                TempArray[i++] = TempByte;
                TempByte = PGMReader.ReadByte();
            }

            TempS = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, i);
            this.mLength = Convert.ToInt32(TempS);

            //read color
            i = 0;
            TempByte = PGMReader.ReadByte();
            while (TempByte != NewLineAsciiCode)
            {
                TempArray[i++] = TempByte;
                TempByte = PGMReader.ReadByte();
            }

            TempS = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, i);
            this.mColor = Convert.ToInt32(TempS);

            //read image data
            byte[] PGMDataBuffer = new byte[this.mWidth * this.mLength];
            int k = 0;
            if (this.mType == "P5")
            {
                //If file is binary, read every byte
                byte[] ReadedByte = PGMReader.ReadBytes(PGMDataBuffer.Length);
                Array.Copy(ReadedByte, PGMDataBuffer, ReadedByte.Length);
            }
            else if (this.mType == "P2")
            {
                //If file is text based every pixel is distinguished by "space" and it has up to 3 chars(255)
                try
                {
                    TempByte = PGMReader.ReadByte();
                    while (TempByte != -1)
                    {
                        i = 0;
                        while (TempByte != NewLineAsciiCode && TempByte != SpaceAsciiCode)
                        {
                            TempArray[i++] = TempByte;
                            TempByte = PGMReader.ReadByte();
                        }

                        TempS = System.Text.ASCIIEncoding.Default.GetString(TempArray, 0, i);
                        i = 0;
                        //TempS contains, string representation of every pixel
                        PGMDataBuffer[k++] = Convert.ToByte(TempS);

                        TempByte = PGMReader.ReadByte();
                        if (TempByte == NewLineAsciiCode || TempByte == SpaceAsciiCode)
                        {
                            TempByte = PGMReader.ReadByte();
                        }
                    }
                }
                catch (Exception e)
                {
                    //Console.WriteLine(e.InnerException);
                }
            }
            this.mData = PGMDataBuffer;

            PGMReader.Close();
            InputStream.Close();
        }
        private void WritePGM(string _filePath)
        {
            //FileStream OutputStream;
            //if (File.Exists(_filePath))
            //{
            FileStream OutputStream = File.Open(_filePath, FileMode.OpenOrCreate, FileAccess.Write);
            //}
            //else
            //{
            //    OutputStream = File.Create(_filePath);
            //}
            BinaryWriter PGMWriter = new BinaryWriter(OutputStream);

            string PGMInfo = this.Header;
            byte[] PGMInfoBuffer = System.Text.ASCIIEncoding.Default.GetBytes(PGMInfo);
            PGMWriter.Write(PGMInfoBuffer);
            if (this.mType == "P5")
            {
                //File is binary, write complete data
                PGMWriter.Write(this.mData);
            }
            else if (this.mType == "P2")
            {
                byte NewLineAsciiCode = 10;
                byte SpaceAsciiCode = 32;
                int Temp;

                for (int i = 0; i < this.Length * this.Width; i++)
                {
                    //File is text based, convert every byte to text representation followed by "space"
                    Temp = this.mData[i];
                    byte[] TempByteArray = System.Text.ASCIIEncoding.Default.GetBytes(Temp.ToString());

                    PGMWriter.Write(TempByteArray);
                    PGMWriter.Write(SpaceAsciiCode);
                    if (i % this.Width == 0)
                    {
                        PGMWriter.Write(NewLineAsciiCode);
                    }
                }
            }
            PGMWriter.Close();

        }

        public void PrintPGMInfo()
        {
            Console.WriteLine("Type       = " + this.Type.ToString());
            Console.WriteLine("ColorSize  = " + this.ColorSize.ToString());
            Console.WriteLine("Comment    = " + this.Comment);
            Console.WriteLine("Length     = " + this.Length.ToString());
            Console.WriteLine("Width      = " + this.Width.ToString());
        }
    }
}
