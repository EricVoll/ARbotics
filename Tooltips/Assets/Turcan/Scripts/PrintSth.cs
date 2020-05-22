using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PrintSth : MonoBehaviour
{
    // Start is called before the first frame update
    private Transform t1;
    void Awake()
    {
        t1 = transform.parent.GetComponent<Transform>();
        //Debug.Log(t1);
    }
    public void Stuff()
    {
        //print("PrintSth");
        gameObject.transform.SetParent(t1, true);
        EnoughisEnough.hasParent = false;
        OpenPanelWithTimer.IsHovering = false;

        //Destroy(transform.GetComponent<FixedJoint>());
        //Destroy(transform.GetComponent<ConfigurableJoint>());
        //sticking.hasJoint = false;
    }

}
