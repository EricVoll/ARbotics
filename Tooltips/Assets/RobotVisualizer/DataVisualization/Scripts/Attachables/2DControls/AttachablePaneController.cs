using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class AttachablePaneController : MonoBehaviour
{
    [SerializeField] GameObject attachableControlPrefab;
    private Dictionary<string, GameObject> controls= new Dictionary<string, GameObject>();
    // Start is called before the first frame update
    [SerializeField] GameObject attachablePanel;
     [SerializeField] GameObject canvas;

    public void Add(string topic){

        // create the 2d component
         GameObject attachableControl = Instantiate<GameObject>(this.attachableControlPrefab);
        ButtonController bc = attachableControl.GetComponent<ButtonController>();
        attachableControl.transform.SetParent(this.attachablePanel.transform, worldPositionStays: false); 
        // LayoutRebuilder.MarkLayoutForRebuild(this.attachablePanel.transform as RectTransform);
        // LayoutRebuilder.ForceRebuildLayoutImmediate(this.attachablePanel.transform as RectTransform);
        // LayoutRebuilder.MarkLayoutForRebuild(this.canvas.transform as RectTransform);
        bc.Topic = topic;
        // (attachableControl.transform as RectTransform).localScale = new Vector3(1.0f, 1.0f, 1.0f);

        this.controls.Add(topic, attachableControl);
    }

    public void Remove(string topic){

    }
}
