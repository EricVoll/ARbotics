using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages.Standard;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndEffectorTarget : MonoBehaviour
{
    //public RosConnector RosConnector;
    private EndEffectorTargetPublisher publisher;

    public string Topic;
    public bool Publish;

    private float lastPublish;

    // Start is called before the first frame update
    void Start()
    {
        publisher = RosConnector.Instance.gameObject.AddComponent<EndEffectorTargetPublisher>();
        publisher.Topic = Topic;

        var subs = gameObject.AddComponent<SubscribeEndeffectorPublishState>();
        subs.Topic = "/t";
        subs.valueChangedCallBack = (value) => {
            Publish = value == 1;
        }; 
    }

    // Update is called once per frame
    void Update()
    {
        float currentTime = UnityEngine.Time.realtimeSinceStartup;
        if (Publish && currentTime - lastPublish > 1) {
            lastPublish = currentTime;
            publisher.Publish(transform.localPosition, transform.localRotation);
        }
    }
}

public class SubscribeEndeffectorPublishState : Subscriber<RosSharp.RosBridgeClient.Messages.Standard.Int32> {

    public Action<int> valueChangedCallBack;
    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Standard.Int32 message) {
        valueChangedCallBack?.Invoke(message.data);
    }
    protected override void Start() {
        RosConnector.Instance.RosSocket.Subscribe<RosSharp.RosBridgeClient.Messages.Standard.Int32>(Topic, ReceiveMessage, (int)(TimeStep * 1000)); // the rate(in ms in between messages) at which to throttle the topics
    }
}
