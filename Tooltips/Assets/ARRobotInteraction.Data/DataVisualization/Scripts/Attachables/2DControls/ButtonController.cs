using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

// To use this example, attach this script to an empty GameObject.
// Create three buttons (Create>UI>Button). Next, select your
// empty GameObject in the Hierarchy and click and drag each of your
// Buttons from the Hierarchy to the Your First Button, Your Second Button
// and Your Third Button fields in the Inspector.
// Click each Button in Play Mode to output their message to the console.
// Note that click means press down and then release.


namespace ARRobotInteraction.Data
{
public class ButtonController : MonoBehaviour
{
    //Make sure to attach these Buttons in the Inspector
    public Button buttonShow, buttonHide;

    public GameObject label;
    public string Topic;

    void Start()
    {
        //Calls the TaskOnClick/TaskWithParameters/ButtonClicked method when you click the Button
        if(buttonShow != null) buttonShow.onClick.AddListener(TaskShow);
        if(buttonHide != null) buttonHide.onClick.AddListener(TaskHide);
    }

    public void TaskShow()
    {
        //Output this to console when Button1 or Button3 is clicked
        Debug.Log("You have clicked the show button!");
        AttachablesManager.Instance.Show(this.Topic);
    }
    public void TaskHide()
    {
        //Output this to console when Button1 or Button3 is clicked
        Debug.Log("You have clicked the hide button!");
        AttachablesManager.Instance.Hide(this.Topic);
    }

    public void TaskFollow()
    {
        Debug.Log("You have clicked the follow button!");
        AttachablesManager.Instance.Follow(this.Topic);
    }
}
}