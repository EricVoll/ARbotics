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
    /// <para></para>
    /// Do not use this method for Unity objects such as Transform and GameObjects, since a movement will change the id
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
        foreach (Transform directChild in transform)
        {
            if (directChild.name == name) return directChild.gameObject;
        }

        return null;
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

    /// <summary>
    /// Fetches the components in the direct children of this gameobject.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="go"></param>
    /// <returns></returns>
    public static List<T> GetComponentsInDirectChildrenFromGameobject<T>(this GameObject go) where T : UnityEngine.Component
    {
        List<T> list = new List<T>();

        var all = go.GetComponentsInChildren<T>();

        foreach (var item in all)
        {
            if (GameObject.ReferenceEquals(item.transform.parent.gameObject, go))
                list.Add(item);
        }

        return list;
    }

    /// <summary>
    /// see <see cref="GetComponentsInDirectChildrenFromGameobject{T}(GameObject)"/>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="go"></param>
    /// <returns></returns>
    public static List<T> GetComponentsInDirectChildrenFromTransform<T>(this Transform go) where T : UnityEngine.Component
    {
        return go.gameObject.GetComponentsInDirectChildrenFromGameobject<T>();
    }

    /// <summary>
    /// Returns all components which are directly on the GameObject <paramref name="go"/> itself
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="go"></param>
    /// <returns></returns>
    public static List<T> GetComponentsInSelf<T>(this GameObject go) where T : UnityEngine.Component
    {
        List<T> list = new List<T>();

        var all = go.GetComponentsInChildren<T>();

        foreach (var item in all)
        {
            if (GameObject.ReferenceEquals(item.gameObject, go))
                list.Add(item);
        }

        return list;
    }

    /// <summary>
    /// Destroys all object in the list
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="items"></param>
    public static void DestroyAll<T>(List<T> items) where T : UnityEngine.Object
    {
        for (int i = items.Count - 1; i >= 0; i--)
        {
            Debug.Log($"Destroyed GameObject {items[i].name}");
            Object.Destroy(items[i]);
        }
    }
    /// <summary>
    /// see <see cref="DestroyAll{T}(List{T})"/>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="items"></param>
    public static void DestroyAll<T>(IEnumerable<T> items) where T : UnityEngine.Object
    {
        DestroyAll(items.ToList());
    }
}
