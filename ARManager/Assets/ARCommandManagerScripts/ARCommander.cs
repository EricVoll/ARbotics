using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARCommander : MonoBehaviour
{
    public void Awake()
    {
        UrdfSyncher.InitializeUrdfSyncher();
    }

    public UrdfSyncher urdfSyncher;

    Dictionary<string, UrdfSyncher> currentSceneDicionary = new Dictionary<string, UrdfSyncher>();

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
    //All available devices that are requestable by the Unity side.
    public List<AvailableRobot> availableDevices;

    //All devices that should be displayed
    public List<DeviceUrdf> devices;
}

public class AvailableRobot
{
    //FriendlyName for UI
    public string name;
    //Id to request the publisher to be started
    public string id;
    //URL where the Stationary Side will publish the urdf contents
    public string publishingServerUrl; 
}

public class DeviceUrdf
{
    //URDF content of the device
    public string data;
    //The ID of the current instance
    public string id;                  
}

