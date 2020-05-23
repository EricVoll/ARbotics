using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ActivationWithButton : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject Panel;
    bool state;
    int counter = 0;
    public void OpenPanel()
    {
        if(Panel!=null)
        {
            counter++;

            state = Panel.activeSelf;

            if (state==true & counter % 2 == 0)
            {
                Panel.SetActive(false);
            }
            if (state == false & counter % 2 == 1)
            {
                //state = !state;
                Panel.SetActive(true);
            }
        }
    }
}
