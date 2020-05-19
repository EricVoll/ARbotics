using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIHandler : MonoBehaviour
{
    public GameObject robUIPrefab;
    public GameObject robUIContainer;
    public void ShowRobots(List<string> names, Action<string> spawnCallBack)
    {
        foreach (var robotName in names)
        {
            GameObject go = Instantiate(robUIPrefab);
            go.GetComponent<RobotEntryScript>().Init(robotName, spawnCallBack);
            go.transform.SetParent(robUIContainer.transform);
        }
    }
}
