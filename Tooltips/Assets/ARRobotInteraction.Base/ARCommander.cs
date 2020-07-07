using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using ARRobotInteraction.Data;


namespace ARRobotInteraction.Base
{
    public class ARCommander : MonoBehaviour
    {
        public void Awake()
        {
            if (UIHandler != null)
                RosSharp.Output.SetHandlers(UIHandler.ShowText, UIHandler.ShowError);
            UrdfSyncher.InitializeUrdfSyncher();
        }

        public void Start()
        {
            InitializeAvailableComponents();
            Application.logMessageReceived += Application_logMessageReceived;
        }

        /// <summary>
        /// Hook into Unity's Logs
        /// Called whenever an Log or Error is printed to Debug Console
        /// </summary>
        /// <param name="condition"></param>
        /// <param name="stackTrace"></param>
        /// <param name="type"></param>
        private void Application_logMessageReceived(string condition, string stackTrace, LogType type)
        {
            if (publishDebugLogsViaRos)
                this.transform.GetComponent<DebugPublisher>()?.SendDebugMsg(condition, stackTrace, type);
        }

        /// <summary>
        /// All Robots will be placed in this gameobject
        /// </summary>
        public GameObject SceneContainerObject;

        /// <summary>
        /// The prefab that is used to instantiate new robots
        /// </summary>
        public GameObject RobotContainerPrefab;

        /// <summary>
        /// a dictionary holding all id to their robot Synchers
        /// </summary>
        readonly Dictionary<int, UrdfSyncher> currentSceneDicionary = new Dictionary<int, UrdfSyncher>();

        /// <summary>
        /// The UI handler responsible for showing hte spawn buttons
        /// </summary>
        public UIHandler UIHandler;

        /// <summary>
        /// If true, there is a hook into the debuglogs and they rae published via ROS
        /// </summary>
        public bool publishDebugLogsViaRos = true;

        public void ReceiveUnfilteredMessage(RunningInstance[] allRunningInstances)
        {
            ReceiveMessage(FilterList(allRunningInstances, (x) => x.comp.pretty_name));
        }

        /// <summary>
        /// Interprets the received list and spawns / destroys instances of robots
        /// </summary>
        /// <param name="runningInstances">Instances of robots that should be synchronized with the scene</param>
        private void ReceiveMessage(RunningInstance[] runningInstances)
        {
#if DEBUG
            Debug.Log($"Recevied {runningInstances.Length} filtered instances");
#endif
            foreach (var instance in runningInstances)
            {
                if (!currentSceneDicionary.ContainsKey(instance.inst.inst_id))
                {
                    //object does not exist. Create it.
                    UrdfSyncher syncher = new UrdfSyncher(SceneContainerObject, transform.GetComponent<RosConnector>(), instance.comp.pretty_name, RobotContainerPrefab);
                    currentSceneDicionary[instance.inst.inst_id] = syncher;
                }

                currentSceneDicionary[instance.inst.inst_id].SynchUrdf(instance.inst.urdf_dyn);
            }

            //Destroy components that are not present anymore
            for (int i = currentSceneDicionary.Values.Count - 1; i >= 0; i--)
            {
                var currentInstance = currentSceneDicionary.Values.ElementAt(i);
                var currentKey = currentSceneDicionary.Keys.ElementAt(i);
                if (!runningInstances.Any(x => x.inst.inst_id == currentKey))
                {
                    currentInstance.Destroy();
                    currentSceneDicionary.Remove(currentKey);
                }
            }

            AttachablesManager.Instance.ReapplyMaterials();
        }

        /// <summary>
        /// Fetch the available components from the REST-API-Endpoint and display them to the user
        /// with the option of spawning robots
        /// </summary>
        private void InitializeAvailableComponents()
        {
            Action<bool> RobotSpawnSuccessPrinter = (bool success) => Debug.Log($"The Robot request {(success ? "succeeded" : "failed")}!");

            Action<ARCommandAvailableComponentResponse> ProcessAvailableComponentsResponse = (ARCommandAvailableComponentResponse res) =>
            {
                UIHandler?.ShowRobots(
                    res.components.Select(x => x.pretty_name).ToList(),
                    (name) => this.GetComponent<RestCommunicator>().RequestNewComponentIntsance(name, RobotSpawnSuccessPrinter)
                    );
            };

            this.GetComponent<RestCommunicator>().RequestAvailableRobots(ProcessAvailableComponentsResponse);
        }

        /// <summary>
        /// Filters the incoming instances list and removes un-spawnable robots
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="inputArray"></param>
        /// <param name="name_selector"></param>
        /// <returns></returns>
        private T[] FilterList<T>(T[] inputArray, Func<T, string> name_selector)
        {
            string[] ignoreNames = new[] { "ros-sharp-com", "UnityCollision" };

            return inputArray.Where(x => !ignoreNames.Any(y => name_selector(x).Contains(y))).ToArray();
        }
    }
}
