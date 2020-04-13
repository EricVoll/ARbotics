using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UrdfSyncher : MonoBehaviour
{
    /// <summary>
    /// The Root gameobject which will be used to spawn all robots. This will be the parent
    /// </summary>
    public GameObject BaseGameObject;

    /// <summary>
    /// Sets the BaseGameObject which will be used to create and synch all robots coming in
    /// Typically set in editor
    /// </summary>
    /// <param name="obj"></param>
    public void SetBaseGameObject(GameObject obj)
    {
        BaseGameObject = obj;
    }

    /// <summary>
    /// Compares the current devices in the scene with the urdf file and performs adjustments
    /// </summary>
    /// <param name="urdf"></param>
    public void SynchUrdf(string urdf)
    {

    }
}
