using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARCommander : MonoBehaviour
{
    public UrdfSyncher urdfSyncher;

    Dictionary<int, UrdfSyncher> currentSceneDicionary = new Dictionary<int, UrdfSyncher>();

    public GameObject SceneContainerObject;

    public void ReceiveMessage(string _msg)
    {
        var msg = Newtonsoft.Json.JsonConvert.DeserializeObject<ARCommanderMessage>(_msg);

        foreach (var device in msg.devices)
        {
            if (!currentSceneDicionary.ContainsKey(device.id))
            {
                //object does not exist. Create it.
                UrdfSyncher syncher = new UrdfSyncher(SceneContainerObject);
                currentSceneDicionary[device.id] = syncher;
            }

            currentSceneDicionary[device.id].SynchUrdf(device.data);
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
    public int id;
}
