using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class Base64ImageSubscriber : Subscriber<Messages.Standard.String>
    {
        public MeshRenderer meshRenderer;

        private Texture2D texture2D;
        private byte[] imageData;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
            texture2D = new Texture2D(1, 1);
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }
        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Standard.String msg)
        {
            imageData = Convert.FromBase64String(msg.data);
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            texture2D.LoadImage(imageData);
            texture2D.Apply();
            meshRenderer.material.SetTexture("_MainTex", texture2D);
            isMessageReceived = false;
        }

    }
}
