using RosSharp.Urdf;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AttachedValue : MonoBehaviour
{
    public AttachableComponent<IAttachableComponent> AttachedComponent { get; set; }
    public Link ParentLink { get; set; }
}
