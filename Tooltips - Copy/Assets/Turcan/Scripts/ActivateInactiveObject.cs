using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class ActivateInactiveObject : MonoBehaviour
{
    public GameObject ToBeActivatedb;
    public Toggle myToggle;
    // Start is called before the first frame update
    public void ActivateInactiveObj()
    {
           if (CameraPropertiesChange.IsPhysicalProp==true)
        {
            ToBeActivatedb.SetActive(true);
        }
        else if (CameraPropertiesChange.IsPhysicalProp == false)
        {
            ToBeActivatedb.SetActive(false);
        }

        if (ToBeActivatedb.tag == "Rays")
        {
            if (myToggle.isOn)
            { ToBeActivatedb.SetActive(true); }
            if(!myToggle.isOn)
            {
                ToBeActivatedb.SetActive(false);
            }
        }
    }
}
