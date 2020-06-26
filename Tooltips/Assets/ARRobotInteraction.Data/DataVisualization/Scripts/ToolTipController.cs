using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARRobotInteraction.Data
{
public class ToolTipController : MonoBehaviour
{

    public Renderer ImageRenderer;

    private string topic;
    public string Topic
    {
        get
        {
            return this.topic;
        }
        set
        {
            this.topic = value;
        }
    }

    public void TaskShow()
    {
        //Output this to console when Button1 or Button3 is clicked
        Debug.Log("You have clicked the show button!");
        AttachablesManager.Instance.Show(this.topic);
    }
    public void TaskHide()
    {
        //Output this to console when Button1 or Button3 is clicked
        Debug.Log("You have clicked the hide button!");
        AttachablesManager.Instance.Hide(this.topic);
    }

    public void TaskFollow()
    {
        Debug.Log("You have clicked the follow button!");
        AttachablesManager.Instance.Follow(this.topic);
    }
}

}