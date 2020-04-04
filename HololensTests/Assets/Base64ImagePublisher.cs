using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Base64ImagePublisher : Publisher<Messages.Standard.String>
    {
        public Camera ImageCamera;
        public string FrameId = "Camera";
        public int resolutionWidth = 640;
        public int resolutionHeight = 480;
        [Range(0, 100)]
        public int qualityLevel = 50;

        private Messages.Standard.String message;
        private Texture2D texture2D;
        private Rect rect;

        protected override void Start()
        {
            base.Start();
            InitializeGameObject();
            InitializeMessage();
            Camera.onPostRender += UpdateImage;
        }

        private void UpdateImage(Camera _camera)
        {
            if (texture2D != null && _camera == this.ImageCamera)
                UpdateMessage();
        }

        private void InitializeGameObject()
        {
            texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
            rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
            ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        }

        private void InitializeMessage()
        {
            message = new Messages.Standard.String(){
                data = "",
            };
        }

        private void UpdateMessage()
        {
            texture2D.ReadPixels(rect, 0, 0);
            message.data = System.Convert.ToBase64String(texture2D.EncodeToJPG(qualityLevel));
            Publish(message);
        }

    }
}
