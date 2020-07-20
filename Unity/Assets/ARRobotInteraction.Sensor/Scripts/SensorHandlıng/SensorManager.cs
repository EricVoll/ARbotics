/*
* MIT License
* 
* Copyright (c) 2020 Eric Vollenweider, Jonas Frey, Raffael Theiler, Turcan Tuna
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

using UnityEngine;
using Microsoft.MixedReality.Toolkit.Diagnostics;
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of controlling all instances of lidars/cameras/etc.
/// </summary>

namespace ARRobotInteraction.Sensor
{
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
}

public enum SensorType { Camera, Lidar2D, Lidar3D }
public enum RosMessageTypes { Image, PointCloud }