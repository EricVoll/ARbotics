using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit.RobotVisualizer;

public class AttachablesManager : Singleton<AttachablesManager>
{

    [SerializeField] GameObject externalPlotPrefab;

    [SerializeField] private GameObject attachablePane;
    public GameObject AttachablePane{
        get {return attachablePane;}
        set {
            this.attachablePane = value;
        }
    }
    private AttachablePaneController paneControl;

    private float frequency = 2f;
    private Dictionary<string, AttachableReference> attachables;


    // Start is called before the first frame update
    void Start()
    {
        this.attachables = new Dictionary<string, AttachableReference>();
        this.paneControl  = this.attachablePane.GetComponent<AttachablePaneController>();
        // StartCoroutine(ManagerUpdateLoop());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDestroy() {
        
    }

    public void Subscribe(string topic, GameObject parent){
        Debug.Log($"Subscribing {parent.name} to topic {topic}");
        this.paneControl.Add(topic);

        // create the 3d component
        GameObject attachable = Instantiate<GameObject>(this.externalPlotPrefab);
        ExternalPlotConnector con = attachable.GetComponent<ExternalPlotConnector>();
        con.Target = parent;
        attachable.transform.SetParent(parent.transform);

        Vector3 offset = new Vector3(0.0f, 0.1f, 0.0f);
        attachable.transform.position = parent.transform.position + offset;
        
        // create the reference object
        AttachableReference aref;
        aref.attachable = attachable;
        aref.parent = parent;

        this.attachables.Add(topic, aref);
    }

    public void Show(string topic){
        if(this.attachables.ContainsKey(topic)){
            AttachableReference aref = this.attachables[topic];
            aref.attachable.SetActive(true);
            aref.attachable.GetComponent<Renderer>().enabled = true;
        }
    }


    public void Hide(string topic){
        if(this.attachables.ContainsKey(topic)){
            AttachableReference aref = this.attachables[topic];
            aref.attachable.SetActive(false);
            aref.attachable.GetComponent<Renderer>().enabled = false;
        }

    }


    public void Unsubscribe(string topic){
        AttachableReference aref = this.attachables[topic];
        this.attachables.Remove(topic);
        

    }

    IEnumerator ManagerUpdateLoop()
    {
        while(true){
            yield return new WaitForSeconds(this.frequency);
            Debug.Log("Topic Processing Loop");
        }
    }
}


public struct AttachableReference {

    public GameObject parent;
    public GameObject attachable;

}
