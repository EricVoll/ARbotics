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

    // Start is called before the first frame update
    void Start()
    {
        publisher = RosConnector.gameObject.AddComponent<EndEffectorTargetPublisher>();
        publisher.Topic = Topic;
    }

    // Update is called once per frame
    void Update()
    {
        if (Publish)
            publisher.Publish(transform.position, transform.rotation);
    }
}
