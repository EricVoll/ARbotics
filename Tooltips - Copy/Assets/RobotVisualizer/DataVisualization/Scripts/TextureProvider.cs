using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TextureProvider : MonoBehaviour
{
    public enum ImageSourceType { ros, http, material }
    private ImageSourceType type;
    private Renderer TargetRenderer;

    /// <summary>
    /// Use this for Ros messages
    /// </summary>
    /// <param name="topicOrUrl"></param>
    /// <param name="targetRenderer"></param>
    /// <param name="type"></param>
    public void InitRosSource(string topicOrUrl, Renderer targetRenderer)
    {
        TargetRenderer = targetRenderer;
        type = ImageSourceType.ros;
        var c = gameObject.AddComponent<RosSharp.RosBridgeClient.Base64ImageSubscriber>();
        c.Topic = topicOrUrl;
        c.meshRenderer = targetRenderer;
    }

    /// <summary>
    /// Use this for Camera sources
    /// </summary>
    /// <param name="material"></param>
    /// <param name="targetRenderer"></param>
    public void InitCameraSource(Material material, Renderer targetRenderer)
    {
        type = ImageSourceType.material;
        TargetRenderer = targetRenderer;
        targetRenderer.material = material;
    }

}