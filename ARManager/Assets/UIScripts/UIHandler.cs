using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIHandler : MonoBehaviour
{
    public ARCommander Commander;

    public TextAsset TurtleBot;
    public TextAsset TurtleBotExtended;
    public TextAsset TurtleBotHalf;
    public TextAsset TurtleBotPluginLess;
    public TextAsset SingleNode;
    public TextAsset SingleNodeExtended;
    public GameObject test;
    public void SendFullTurtleBot()
    {
        SendMsg(TurtleBotHalf);
    }
    public void SendExtendedTurtleBot()
    {
        SendMsg(TurtleBotPluginLess);
    }

    public void SendSingleNode()
    {
        SendMsg(SingleNode);
    }
    public void SendExtendedNode()
    {
        SendMsg(SingleNodeExtended);
    }

    private void SendMsg(TextAsset asset)
    {
        if (asset != null)
            Commander.ReceiveMessage(MockCommanderMessage(asset.text));
    }

    bool isScond = false;
    private string MockCommanderMessage(string urdf)
    {
        ARCommanderMessage mock = new ARCommanderMessage()
        {
            devices = new List<DeviceUrdf>()
            {
                new DeviceUrdf(){ data = urdf, id = "1" }
            },
            availableDevices = new List<AvailableRobot>()
        };

        //if (isScond)
        //{
        //    mock.devices.Add(new ObjectUrdf() { data = urdf, id = 2 });
        //}


        string json = Newtonsoft.Json.JsonConvert.SerializeObject(mock);
        isScond = true;
        return json;
    }
}
