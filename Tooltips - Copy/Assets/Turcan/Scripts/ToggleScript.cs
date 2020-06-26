using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleScript : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject Object;
    int counter = 0;
    public void ToggleState()
    {

        counter++;
        bool state = Object.GetComponent<VisionCone>().isActiveAndEnabled;

        if (state == true & counter % 2 == 0)
        {
            Object.SetActive(false);
        }
        if (state == false & counter % 2 == 1)
        {
            //state = !state;
            Object.SetActive(true);
            //print(state);//.enabled = !Object.GetComponent<VisionCone>().enabled; //this changes the state from on to off and vice-versa
            //Object.GetComponent<VisionCone>().enabled = false;
            //state = Object.GetComponent<VisionCone>().isActiveAndEnabled;
            //print(state);
        }
    }
}
