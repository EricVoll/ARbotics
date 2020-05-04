using RosSharp.Urdf;
using RosSharp.Urdf.Attachables;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AttachedDataSynchronizer
{
    private static AttachedDataSynchronizer instance;
    public static AttachedDataSynchronizer Instance
    {
        get
        {
            if (instance == null) instance = new AttachedDataSynchronizer();
            return instance;
        }
    }

    internal void HandleAttachedComponent(GameObject linkObject, AttachableComponent<IAttachableComponent> attachedComponent, Link link)
    {
        if (Utils.FindChildOrCreateWithComponent(linkObject.transform, attachedComponent.component.name, out GameObject attachedComponentGO, out AttachedValue v))
        {
            if (attachedComponent.component is AttachedDataValue)
            {
                //Instaniate GameObject with Raffi's ToolTip Manager 
            }
        }
    }
}
