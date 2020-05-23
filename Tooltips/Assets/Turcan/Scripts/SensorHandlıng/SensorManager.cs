using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Diagnostics;
/// <summary>
/// Is responsible of controlling all instances of lidars/cameras/etc.
/// </summary>


public class SensorManager : MonoBehaviour
{
    public static SensorManager instance;
    public VisualProfilerControl visualProfilerControlPrefab;

    public void Awake()
    {
        instance = this;
        visualProfilerControlPrefab.ToggleProfiler();
        visualProfilerControlPrefab.SetProfilerVisibility(false);
    }

    /// <summary>
    /// This is the parent of our sensors if they are not sticked to the robots
    /// </summary>
    public GameObject DefaultSensorParent;


    public GameObject CameraPrefab;
    public GameObject Lidar2DPrefab;
    public GameObject Lidar3DPrefab;


    /// <summary>
    /// Returns an instance of the specified sensor
    /// </summary>
    /// <param name="type"></param>
    /// <param name="streamBackTopic"></param>
    /// <param name="msg"></param>
    /// <returns></returns>
    public void CreateSensor(SensorType type, string streamBackTopic = null, RosMessageTypes? msg = null)
    {
        if (CurrentlyVisible != null)
        {
            Destroy(CurrentlyVisible);
        }

        //type = ReadDropdown.captiontes;
        switch (type)
        {
            case SensorType.Camera:
                //create GameObjects from Prefabs
                CurrentlyVisible = Instantiate(CameraPrefab);
                break;
            case SensorType.Lidar2D:
                CurrentlyVisible = Instantiate(Lidar2DPrefab);
                var c = CurrentlyVisible.GetComponentInChildren<VisionCone>();
                c.ExtractPointCallBack = ExtractPoints;
                break;
            case SensorType.Lidar3D:
                CurrentlyVisible = Instantiate(Lidar3DPrefab);
                CurrentlyVisible.GetComponentInChildren<LidarSensor>().ExtractPointCallBack = ExtractPoints;
                break;
            default:
                break;
        }

        CurrentlyVisible.transform.SetParent(DefaultSensorParent.transform, true);
    }

    private void ExtractPoints(Vector3[] points)
    {
        //Send to Ros#
    }

    GameObject CurrentlyVisible;
}


public enum SensorType { Camera, Lidar2D, Lidar3D }
public enum RosMessageTypes { Image, PointCloud }