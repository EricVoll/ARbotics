using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UIInputHandler : MonoBehaviour
{

    public MoveItPublisher moveItPublisher;
    public Toggle toggle;

    public void SendOnce()
    {
        if(moveItPublisher != null)
        {
            moveItPublisher.UpdateAndPublishMessage(); 
        }
    }

    public void SetAutoToggleMode(bool value)
    {
        if(moveItPublisher != null && toggle != null)
        {
            moveItPublisher.AutoSend = toggle.isOn;
        }
    }
}
