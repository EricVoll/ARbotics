using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace ARRobotInteraction.Data
{
public class AttachablePaneController : MonoBehaviour
{
    [SerializeField] GameObject attachableControlPrefab;
    private Dictionary<string, GameObject> controls= new Dictionary<string, GameObject>();
    // Start is called before the first frame update
    [SerializeField] GameObject attachablePanel;
     [SerializeField] GameObject canvas;

    public void Add(string topic){

        // create the 2D component
         GameObject attachableControl = Instantiate<GameObject>(this.attachableControlPrefab);
        ButtonController bc = attachableControl.GetComponent<ButtonController>();
        attachableControl.transform.SetParent(this.attachablePanel.transform, worldPositionStays: false); 
        bc.Topic = topic;

        this.controls.Add(topic, attachableControl);
    }

    public void Remove(string topic){

    }
}
}