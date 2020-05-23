using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndEffectorTarget : MonoBehaviour
{
    public RosConnector RosConnector;
    private EndEffectorTargetPublisher publisher;
    public string Topic;
    public bool Publish;

    private float lastPublish;

    // Start is called before the first frame update
    void Start()
    {
        publisher = RosConnector.gameObject.AddComponent<EndEffectorTargetPublisher>();
        publisher.Topic = Topic;
    }

    // Update is called once per frame
    void Update()
    {
        float currentTime = Time.realtimeSinceStartup;
        if (Publish && currentTime - lastPublish > 1) {
            lastPublish = currentTime;
            var unityToGazebo = Quaternion.Euler(new Vector3(0,0,0));
            publisher.Publish(transform.position, transform.rotation * unityToGazebo);
        }
    }
}
