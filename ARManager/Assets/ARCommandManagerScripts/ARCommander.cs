using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARCommander : MonoBehaviour
{
    public void Awake()
    {
        UrdfSyncher.InitializeUrdfSyncher();

        InitializeAvailableComponents();
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

    private void InitializeAvailableComponents()
    {
        var availableComponents = this.GetComponent<RestCommunicator>().RequestAvailableRobots();
        foreach (var item in availableComponents.components)
        {
            Debug.Log(item.pretty_name);
        }
    }

}
