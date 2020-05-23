using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugPublisher : Publisher<RosSharp.RosBridgeClient.Messages.Standard.String>
{
    public void SendDebugMsg(string msg, string stackTrace, LogType type)
    {
        var message = new RosSharp.RosBridgeClient.Messages.Standard.String()
        {
            data = $"msg: {msg}\nStacktrace: {stackTrace}\nLogType: {type}"
        };
        Publish(message);
    }
}
