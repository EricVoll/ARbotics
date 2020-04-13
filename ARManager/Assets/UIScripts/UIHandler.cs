using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIHandler : MonoBehaviour
{
    public ARCommander Commander;

    public TextAsset TurtleBot;
    public TextAsset TurtleBotExtended;
    public TextAsset SingleNode;
    public TextAsset SingleNodeExtended;

    public void SendFullTurtleBot()
    {
        SendMsg(TurtleBot);
    }
    public void SendExtendedTurtleBot()
    {
        SendMsg(TurtleBotExtended);
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


    private string MockCommanderMessage(string urdf)
    {
        ARCommanderMessage mock = new ARCommanderMessage()
        {
            devices = new List<ObjectUrdf>()
            {
                new ObjectUrdf(){ data = urdf }
            },
            availableRobots = new List<AvailableRobot>()
        };
        string json = Newtonsoft.Json.JsonConvert.SerializeObject(mock);

        return json;
    }
}
