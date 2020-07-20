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

    public Func<string, bool> ShouldCreate { get; set; }
    public Action<string, GameObject> CreateCallBack;
    public Action<string> DestroyCallBack;

    internal void HandleAttachedComponent(GameObject linkObject, AttachableComponent<IAttachableComponent> attachedComponent, Link link)
    {
        if (Utils.FindChildOrCreateWithComponent(linkObject.transform, attachedComponent.component.name, out GameObject attachedComponentGO, out AttachedValue v))
        {
            v.AttachedComponent = attachedComponent;
            if (attachedComponent.component is AttachedDataValue av)
            {
                //Instaniate GameObject with Raffi's ToolTip Manager 
                //AttachablesManager.Instance.Subscribe(av.topic, )
                CreateCallBack?.Invoke(av.topic, linkObject);
            }
        }
    }
    internal void RemoveAttachedComponent(AttachableComponent<IAttachableComponent> attachedComponent) {
       if (attachedComponent.component is AttachedDataValue av) {
           // DestroyCallBack?.Invoke(av.topic);
       }
    }
    internal bool ShouldCreateComponent(AttachableComponent<IAttachableComponent> c) {
        if (ShouldCreate == null) return true;
        if (c.component is AttachedDataValue av)
            return ShouldCreate.Invoke(av.topic);
        return true;
    }
}
