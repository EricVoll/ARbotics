using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARCommander : MonoBehaviour
{
    public UrdfSyncher urdfSyncher;

    public void ReceiveMessage(string _msg)
    {
        var msg = Newtonsoft.Json.JsonConvert.DeserializeObject<ARCommanderMessage>(_msg);
        
        foreach (var device in msg.devices)
        {
            urdfSyncher.SynchUrdf(device.data);
        }
    }
}


public class ARCommanderMessage
{
    public List<AvailableRobot> availableRobots = new List<AvailableRobot>();
    public List<ObjectUrdf> devices = new List<ObjectUrdf>();
}

public class AvailableRobot
{
    public string name;
    public string id;
}

public class ObjectUrdf
{
    public string data;
}
