using System.Collections;
using System.Collections.Generic;
using UnityEngine;



namespace RosSharp.RosBridgeClient
{
    public class Base64ImageSubscriber : Subscriber<Messages.Standard.String>
    {
        public Renderer meshRenderer;

        private Texture2D texture2D;
        private byte[] imageData;
        private bool isMessageReceived;

        protected override void Start()
        {
            RosConnector.Instance.RosSocket.Subscribe<Messages.Standard.String>(Topic, ReceiveMessage, (int)(TimeStep * 1000)); // the rate(in ms in between messages) at which to throttle the topics

            texture2D = new Texture2D(1, 1);

            if (meshRenderer != null)
                meshRenderer.material = new Material(Shader.Find("Standard"));
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Standard.String msg)
        {
            imageData = System.Convert.FromBase64String(msg.data);
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (meshRenderer == null) return;

            texture2D.LoadImage(imageData);
            texture2D.Apply();
            meshRenderer.material.SetTexture("_MainTex", texture2D);
            isMessageReceived = false;
        }

    }
}