using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ARCommander : MonoBehaviour
{
    public void Awake()
    {
        UrdfSyncher.InitializeUrdfSyncher();

        InitializeAvailableComponents();
    }

    /// <summary>
    /// All Robots will be placed in this gameobject
    /// </summary>
    public GameObject SceneContainerObject;

    /// <summary>
    /// a dictionary holding all id to their robot Synchers
    /// </summary>
    readonly Dictionary<int, UrdfSyncher> currentSceneDicionary = new Dictionary<int, UrdfSyncher>();

    /// <summary>
    /// The UI handler responsible for showing hte spawn buttons
    /// </summary>
    public UIHandler UIHandler;


    public void ReceiveMessage(RunningInstance[] runningInstances)
    {
        foreach (var instance in runningInstances)
        {
            if (!currentSceneDicionary.ContainsKey(instance.inst.inst_id))
            {
                //object does not exist. Create it.
                UrdfSyncher syncher = new UrdfSyncher(SceneContainerObject, instance.comp.pretty_name);
                currentSceneDicionary[instance.inst.inst_id] = syncher;
            }

            currentSceneDicionary[instance.inst.inst_id].SynchUrdf(instance.inst.urdf_dyn);
        }
    }


    ARCommandAvailableComponentResponse currentResponse;

    private void InitializeAvailableComponents()
    {
        this.GetComponent<RestCommunicator>().RequestAvailableRobots(ProcessResponse);
    }

    private void ProcessResponse(ARCommandAvailableComponentResponse res)
    {
        foreach (var item in res.components)
        {
            Debug.Log(item.pretty_name);
        }

        UIHandler.ShowRobots(res.components.Select(x => x.pretty_name).ToList(), SpawnRobot);
    }

    private void SpawnRobot(string name)
    {
        this.GetComponent<RestCommunicator>().RequestNewComponentIntsance(name, RobotAdded);
    }

    private void RobotAdded(bool success)
    {
        //Request ru
        RequestRunningIntsances();
    }

    //This should not be necessary in the long run.
    private void RequestRunningIntsances()
    {
        this.GetComponent<RestCommunicator>().SendGetRequest<RunningInstance[]>(RestCommunicator.RequestUrl.Instances, RunningInstancesReceived);
    }

    private void RunningInstancesReceived(RunningInstance[] instances)
    {
        string[] ignoreNames = new[] { "ros-sharp-com", "UnityCollision" };

        List<RunningInstance> inst = instances.Where(x => !ignoreNames.Any(y => x.comp.pretty_name.Contains(y))).ToList();

        ReceiveMessage(inst.ToArray());
    }
}
