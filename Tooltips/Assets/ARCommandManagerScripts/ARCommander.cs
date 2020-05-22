using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


namespace ARCommander {
    public class ARCommander : MonoBehaviour {
        public void Awake() {
            InitializeAvailableComponents();

            RosSharp.Output.SetHandlers(UIHandler.ShowText, UIHandler.ShowError);
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

        public void ReceiveUnfilteredMessage(RunningInstance[] allRunningInstances) {
            ReceiveMessage(FilterList(allRunningInstances, (x) => x.comp.pretty_name));
        }

        public void ReceiveMessage(RunningInstance[] runningInstances) {
#if DEBUG
            Debug.Log($"Recevied {runningInstances.Length} filtered instances");
#endif
            foreach (var instance in runningInstances) {
                if (!currentSceneDicionary.ContainsKey(instance.inst.inst_id)) {
                    //object does not exist. Create it.
                    UrdfSyncher syncher = new UrdfSyncher(SceneContainerObject, transform.GetComponent<RosConnector>(), instance.comp.pretty_name, RobotContainerPrefab);
                    currentSceneDicionary[instance.inst.inst_id] = syncher;
                }

                try {
                    currentSceneDicionary[instance.inst.inst_id].SynchUrdf(instance.inst.urdf_dyn);
                }
                catch (Exception ex) {
                    Debug.LogError("Robot Synchronization failed! " + ex.Message);
                    Debug.LogError(ex.StackTrace);
                }
            }

            return;
            for (int i = currentSceneDicionary.Values.Count - 1; i >= 0; i--) {
                var currentInstance = currentSceneDicionary.Values.ElementAt(i);
                var currentKey = currentSceneDicionary.Keys.ElementAt(i);
                if (!runningInstances.Any(x => x.inst.inst_id == currentKey)) {
                    currentInstance.Destroy();
                    currentSceneDicionary.Remove(currentKey);
                }
            }
        }


        ARCommandAvailableComponentResponse currentResponse;

        private void InitializeAvailableComponents() {
            this.GetComponent<RestCommunicator>().RequestAvailableRobots(ProcessResponse);
        }

        private void ProcessResponse(ARCommandAvailableComponentResponse res) {
            foreach (var item in res.components) {
                Debug.Log(item.pretty_name);
            }

            UIHandler?.ShowRobots(res.components.Select(x => x.pretty_name).ToList(), SpawnRobot);
        }

        private void SpawnRobot(string name) {
            this.GetComponent<RestCommunicator>().RequestNewComponentIntsance(name, RobotAdded);
        }

        private void RobotAdded(bool success) {
            if (success)
                Debug.Log("Robot request succeeded!");
            else
                Debug.Log("Robot request failed!");
        }

        //This should not be necessary in the long run.
        private void RequestRunningIntsances() {
            this.GetComponent<RestCommunicator>().SendGetRequest<RunningInstance[]>(RestCommunicator.RequestUrl.Instances, RunningInstancesReceived);
        }

        private void RunningInstancesReceived(RunningInstance[] instances) {
            ReceiveUnfilteredMessage(instances);
        }

        private T[] FilterList<T>(List<T> inputList, Func<T, string> name_selector) {
            return FilterList<T>(inputList.ToArray(), name_selector);
        }
        private T[] FilterList<T>(T[] inputArray, Func<T, string> name_selector) {
            string[] ignoreNames = new[] { "ros-sharp-com", "UnityCollision" };

            return inputArray.Where(x => !ignoreNames.Any(y => name_selector(x).Contains(y))).ToArray();
        }
    }
}
