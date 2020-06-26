using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;


namespace ARRobotInteraction.Data {

public class AttachablesManager : Singleton<AttachablesManager> {

    [SerializeField] GameObject externalPlotPrefab;

    [SerializeField] private GameObject attachablePane;
    public GameObject AttachablePane {
        get { return attachablePane; }
        set {
            this.attachablePane = value;
        }
    }

    public Material activeMaterial;
    public Material inactiveMaterial;

    private AttachablePaneController paneControl;
    private GameObject attachablesRoot;

    private float frequency = 2f;
    private Dictionary<string, AttachableReference> attachables;


    // Start is called before the first frame update
    void Awake() {
        this.attachables = new Dictionary<string, AttachableReference>();
        this.attachablesRoot = new GameObject("Attachables");
        this.attachablesRoot.transform.SetParent(transform);

        if(attachablePane != null)
        this.paneControl = this.attachablePane.GetComponent<AttachablePaneController>();
        // StartCoroutine(ManagerUpdateLoop());
    }

    // Update is called once per frame
    void Update() {

    }

    private void OnDestroy() {

    }

    public void Subscribe(string topic, GameObject parent) {
        if (!this.attachables.ContainsKey(topic)) {
            Debug.Log($"Subscribing {parent.name} to topic {topic}");
            //this.paneControl.Add(topic);


            var visuals = parent.transform.Find("Visuals");
            if (visuals != null) {
                var c = visuals.GetComponentsInChildren<Renderer>();
                parent = c[0].gameObject;
            };

            //foreach (var render in r) {
            //    render.material = activeMaterial;
            //}


            // modify the parent's interactability (collision with the pointer) 
            parent.GetComponent<Renderer>().material = activeMaterial;
            NearInteractionTouchable touchable = parent.AddComponent<NearInteractionTouchable>();
            BoxCollider collider = parent.AddComponent<BoxCollider>();
            touchable.SetTouchableCollider(collider);

            // instanciate an interactable that processes all kinds of events to the respective function.
            Interactable interactable = attachInteractible(topic, parent);
            PressableButton button = parent.AddComponent<PressableButton>();

            // route physical presses to the interactable
            PhysicalPressEventRouter physicalEvents = parent.AddComponent<PhysicalPressEventRouter>();
            physicalEvents.routingTarget = interactable;

            //PointerHandler ph = parent.AddComponent<PointerHandler>();
            //new PointerUnityEvent()
            //ph.OnPointerClicked(() => this.Toggle(topic));

            // create the 3d component
            GameObject attachable = Instantiate<GameObject>(this.externalPlotPrefab);

            ToolTipController toolTipController = attachable.GetComponent<ToolTipController>();
            toolTipController.Topic = topic;
            var texture = attachable.AddComponent<TextureProvider>();
            texture.InitRosSource(topic, toolTipController.ImageRenderer);
            ExternalPlot eplot = attachable.GetComponent<ExternalPlot>();
            eplot.ToolTipText = topic;
            eplot.FontSize = 20;

            ExternalPlotConnector con = attachable.GetComponent<ExternalPlotConnector>();
            con.Target = parent;
            attachable.transform.SetParent(this.attachablesRoot.transform);

            Vector3 offset = new Vector3(0.0f, 0.1f, 0.0f);
            attachable.transform.position = parent.transform.position + offset;

            // create the reference object
            AttachableReference aref;
            aref.attachable = attachable;
            aref.parent = parent;
            aref.active = true;
            aref.following = false;

            this.attachables.Add(topic, aref);
        } else {
            this.Show(topic);
        }
    }

    public bool ShouldCreate(string topic) => !attachables.ContainsKey(topic);


    public void ReapplyMaterials() {
        foreach (var aref in attachables.Values) {
            if  (aref.active) {
                aref.parent.GetComponent<Renderer>().material = activeMaterial;
            } else { 
                aref.parent.GetComponent<Renderer>().material = inactiveMaterial;
            }
        }
    }

    private Interactable attachInteractible(string topic, GameObject parent) {
        Interactable i = parent.AddComponent<Interactable>();

        ThemeDefinition theme = ThemeDefinition.GetDefaultThemeDefinition<InteractableColorTheme>().Value;

        i.Profiles = new List<InteractableProfileItem>() {
            new InteractableProfileItem(){
                Themes = new List<Theme>()
                {
                    Interactable.GetDefaultThemeAsset(new List<ThemeDefinition>() {theme})
                },
                Target = parent,
            }
        };

        i.enabled = true;
        i.OnClick.AddListener(() => this.Toggle(topic));
        return i;

    }

    public void Toggle(string topic) {
        if (this.attachables.ContainsKey(topic)) {
            Debug.Log($"Toggling tooltip of topic {topic}");
            AttachableReference aref = this.attachables[topic];
            if (aref.active) {
                this.Hide(topic);
            } else {
                this.Show(topic);
            }
        } else {
            Debug.LogWarning($"Topic {topic} not found");
        }
    }

    public void Show(string topic) {
        if (this.attachables.ContainsKey(topic)) {
            AttachableReference aref = this.attachables[topic];
            aref.parent.GetComponent<Renderer>().material = activeMaterial;
            aref.attachable.SetActive(true);
            aref.active = true;
            aref.attachable.GetComponent<Renderer>().enabled = true;
            this.attachables[topic] = aref;
        } else {
            Debug.LogWarning($"Topic {topic} not found");
        }
    }


    public void Hide(string topic) {
        if (this.attachables.ContainsKey(topic)) {
            AttachableReference aref = this.attachables[topic];
            aref.parent.GetComponent<Renderer>().material = inactiveMaterial;
            aref.attachable.SetActive(false);
            aref.active = false;
            aref.attachable.GetComponent<Renderer>().enabled = false;
            this.attachables[topic] = aref;
        } else {
            Debug.LogWarning($"Topic {topic} not found");
        }

    }


    public void Follow(string topic) {
        if (this.attachables.ContainsKey(topic)) {
            AttachableReference aref = this.attachables[topic];
            if (aref.following) {
                aref.attachable.GetComponent<RadialView>().enabled = false;
            } else {
                aref.attachable.GetComponent<RadialView>().enabled = true;
            }

            aref.following = !aref.following;
            this.attachables[topic] = aref;
        } else {
            Debug.LogWarning($"Topic {topic} not found");
        }
    }


    public void Unsubscribe(string topic) {
        AttachableReference aref = this.attachables[topic];
        this.attachables.Remove(topic);
        // TODO: destroy game object
        Destroy(aref.attachable);
        Destroy(aref.parent.GetComponent<PressableButton>());
        Destroy(aref.parent.GetComponent<BoxCollider>());
        Destroy(aref.parent.GetComponent<NearInteractionTouchable>());
    }

    IEnumerator ManagerUpdateLoop() {
        while (true) {
            yield return new WaitForSeconds(this.frequency);
            // may be needed later
            // Debug.Log("Topic Processing Loop");
        }
    }
}


public struct AttachableReference {

    public bool active;
    public bool following;
    public GameObject parent;
    public GameObject attachable;

}
}