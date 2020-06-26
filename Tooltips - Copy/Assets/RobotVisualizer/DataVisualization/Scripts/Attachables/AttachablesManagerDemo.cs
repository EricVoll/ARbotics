using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AttachablesManagerDemo : MonoBehaviour
{


    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Attachables Demo");
        // Hypothetical Knowledge from URDF
        GameObject robot = GameObject.Find("Robot").gameObject;
        GameObject upperArm = GameObject.Find("EndNozzle").gameObject;
        AttachablesManager am = AttachablesManager.Instance;
        am.Subscribe("test/robot", robot);
        am.Subscribe("test/arm", upperArm);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
