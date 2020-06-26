using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;



namespace ARRobotInteraction.Base
{
    public class RobotEntryScript : MonoBehaviour
    {

        public void Init(string robName, Action<string> spawnCallBack)
        {
            this.GetComponentInChildren<Text>().text = robName;
            SpawnCallBack = spawnCallBack;
            PrettyName = robName;
        }
        public Action<string> SpawnCallBack;
        public string PrettyName { get; set; }
        public void SpawnRobot()
        {
            SpawnCallBack(PrettyName);
        }
    }
}