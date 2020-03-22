using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LitSetup : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Libarary Logger values set!");
        Assets.RosSharp.Scripts.Logger.setLogDelegate((o) => Debug.Log(o));
        Assets.RosSharp.Scripts.Logger.setErrorDelegate((o) => Debug.LogError(o));
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
