using System;
using System.Collections.Generic;
using System.Text;
using System.IO;
using System.Drawing;

namespace main
{
    class MapTool
    {
        private Bitmap _map;
        private Size _boxSize;
        private float _scale;
        private bool _isMapInit;
        public MapTool()
        {
            _scale = 1;
            IsMapInit = false;
        }
        public MapTool(string mapPath, Size boxSize)
        {
            _scale = 1;
            Map = (Bitmap)Image.FromFile(mapPath);
            BoxSize = boxSize;
            IsMapInit = true;
            AutoScale();
        }
        public void SetMap(string mapPath, Size boxSize)
        {
            Map = (Bitmap)Image.FromFile(mapPath);
            BoxSize = boxSize;
            IsMapInit = true;
            AutoScale();
        }
        private void AutoScale()
        {
            if(IsMapInit)
                _scale = GetScale(Map.Size, BoxSize);
        }
        private float GetScale(Size mapSize, Size boxSize)
        {
            float scale = 1.0f;
            if (mapSize.Width > boxSize.Width || mapSize.Height > boxSize.Height)
            {
                if ((float)boxSize.Width / mapSize.Width < (float)boxSize.Height / mapSize.Height)
                {
                    scale = (int)(((float)boxSize.Width / mapSize.Width)*10)/10.0f;
                }
                else
                {
                    scale = (int)(((float)boxSize.Height / mapSize.Height) * 10) / 10.0f;
                }
            }
            return scale;
        }
        public void ZoomIn()
        {
            if (IsMapInit)
            {
                Scale += GetScale(Map.Size, BoxSize);
            }
        }
        public void ZoomOut()
        {
            if (IsMapInit)
            {
                Scale -= GetScale(Map.Size, BoxSize);
            }
        }

        public Bitmap Map
        {
            get { return _map; }
            private set { _map = value; }
        }
        public Image ThumbnailMap
        {
            get 
            {
                if (IsMapInit)
                {
                    return Map.GetThumbnailImage((int)(Map.Width * Scale), (int)(Map.Height * Scale), null, IntPtr.Zero);
                }
                else
                    return null;
            }
        }
        public Size BoxSize 
        {
            get { return _boxSize; }
            private set { _boxSize = value; }
        }
        public float Scale 
        {
            get { return _scale; }
            private set 
            {
                if (value < GetScale(Map.Size, BoxSize)) _scale = GetScale(Map.Size, BoxSize);
                else if (value > 5) _scale = 5;
                else _scale = value;
            }
        }
        public bool IsMapInit
        {
            get { return _isMapInit; }
            private set { _isMapInit = value; }
        }
    }
}
