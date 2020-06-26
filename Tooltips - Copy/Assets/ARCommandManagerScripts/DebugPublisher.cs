using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DebugPublisher : Publisher<RosSharp.RosBridgeClient.Messages.Standard.String>
{
    public void SendDebugMsg(string msg, string stackTrace, LogType type)
    {
        var obj = new {
            content = msg,
            stack = stackTrace,
            t = type
        };

        var json = JsonConvert.SerializeObject(obj);

        var message = new RosSharp.RosBridgeClient.Messages.Standard.String()
        {
            data = json
        };

        Publish(message);
    }
}
