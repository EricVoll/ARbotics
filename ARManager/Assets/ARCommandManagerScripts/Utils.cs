using RosSharp;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class Utils
{
    /// <summary>
    /// Generates a value from the object to compare it with other objects for equality on a non-reference base.
    /// <para></para>
    /// The resulting value hopefully is unique.
    /// <para></para>
    /// If information about the parent exists, it includes it in its calculation
    /// 
    /// </summary>
    /// <param name="obj"></param>
    /// <returns></returns>
    public static string GenerateNonReferenceID(this object obj, object parent = null)
    {
        string json = Newtonsoft.Json.JsonConvert.SerializeObject(obj);
        if (parent != null)
        {
            return $"{obj.GetType()}_{parent.GetType()}_{json.GetHashCode()}";
        }
        return $"{obj.GetType()}_{json.GetHashCode()}";
    }


    /// <summary>
    /// Returns the first GameObject with the name if any is found. If no children exist or the named does not exist it returns null
    /// </summary>
    /// <param name="transform"></param>
    /// <returns></returns>
    public static GameObject FindChildIfExists(this Transform transform, string name)
    {
        var children = transform.GetComponentsInChildren<Transform>();
        if (children == null || children.Length == 0) return null;

        var child = children.FirstOrDefault(x => x.name == name);
        if (child != null) return child.gameObject;
        else return null;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="transform"></param>
    /// <param name="name"></param>
    /// <param name="created"></param>
    /// <returns></returns>
    public static bool FindChildOrCreate(this Transform transform, string name, out GameObject child)
    {
        bool created = false;
        GameObject obj = FindChildIfExists(transform, name);
        if (obj == null)
        {
            Debug.Log("Created a child named " + name);
            obj = new GameObject(name);
            obj.transform.SetParentAndAlign(transform);
            created = true;
        }
        child = obj;
        return created;
    }

    /// <summary>
    /// Looks for a gameobject child with the specified name and creates one if none is found. After creation it also appends the defined component
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="transform"></param>
    /// <param name="name"></param>
    /// <param name="child"></param>
    /// <returns></returns>
    public static bool FindChildOrCreateWithComponent<T>(this Transform transform, string name, out GameObject child, out T component) where T : UnityEngine.Component
    {
        bool createdComponent = false;
        if (transform.FindChildOrCreate(name, out child))
        {
            child.AddComponent<T>();
            createdComponent = true;
        }
        component = child.GetComponent<T>();
        return createdComponent;
    }

    /// <summary>
    /// Adds the component to the transform if it does not exist yet.
    /// Returns true if it was added, false if it existed already.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="transform"></param>
    /// <returns></returns>
    public static bool AddComponentIfNotExists<T>(this GameObject transform, out T component) where T : UnityEngine.Component
    {
        component = transform.GetComponent<T>();
        if (component != null)
        {
            return false;
        }
        else
        {
            component = transform.AddComponent<T>();
            return true;
        }
    }

    /// <summary>
    /// See <see cref="AddComponentIfNotExists{T}(GameObject, out T)"/>
    /// <para>This overload does not pass back a reference to the component added</para>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="transform"></param>
    /// <returns></returns>
    public static bool AddComponentIfNotExists<T>(this GameObject transform) where T : UnityEngine.Component
    {
        return transform.AddComponentIfNotExists<T>(out T _);
    }
}
