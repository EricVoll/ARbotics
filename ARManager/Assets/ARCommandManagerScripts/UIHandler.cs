using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIHandler : MonoBehaviour
{
    public GameObject robUIPrefab;
    public GameObject robUIContainer;
    public GameObject logContainer;

    public GameObject logPrefab;
    public GameObject errorPrefab;

    public void ShowRobots(List<string> names, Action<string> spawnCallBack)
    {
        foreach (var robotName in names)
        {
            GameObject go = Instantiate(robUIPrefab);
            go.GetComponent<RobotEntryScript>().Init(robotName, spawnCallBack);
            go.transform.SetParent(robUIContainer.transform);
        }

        typeof(Test).GetField("test").GetRawConstantValue();
        typeof(Test).GetField("test2").GetRawConstantValue();
    }

    int logCounter = 0;
    public void ShowText(string text)
    {
        GameObject log = GameObject.Instantiate(logPrefab);
        log.GetComponent<Text>().text = logCounter++ + text;
        log.transform.SetParent(logContainer.transform);
    }
    public void ShowError(string text)
    {
        GameObject log = GameObject.Instantiate(errorPrefab);
        log.GetComponent<Text>().text = logCounter++ + text;
        log.transform.SetParent(logContainer.transform);
    }

    public class Test
    {
        public string test = "myTest";
        public const string test2 = "myTest2";
    }
}
