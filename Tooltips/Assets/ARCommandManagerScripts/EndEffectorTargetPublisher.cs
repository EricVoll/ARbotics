using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EndEffectorTargetPublisher : Publisher<RosSharp.RosBridgeClient.Messages.Geometry.Pose>
{
    public void Publish(Vector3 position, Quaternion orientation)
    {
        var m = new RosSharp.RosBridgeClient.Messages.Geometry.Pose()
        {
            position = new RosSharp.RosBridgeClient.Messages.Geometry.Point()
            {
                x = position.x,
                y = position.y,
                z = position.z
            },
            orientation = new RosSharp.RosBridgeClient.Messages.Geometry.Quaternion()
            {
                w = orientation.w,
                x = orientation.x,
                y = orientation.y,
                z = orientation.z
            }
        };
        Publish(m);
    }
}
