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
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of Changing the camera properties.
/// </summary>
namespace ARRobotInteraction.Sensor
{
    public class CameraPropertiesChange : MonoBehaviour
    {
        readonly float InitialDepth = 0.0f;
        public Camera Cam;
        public Slider slider;
        public Slider FOVslider;
        public Toggle toggle;
        public Dropdown dropdown;
        int filmHeight = 24;
        int filmWidth = 36;
        public static bool IsPhysicalProp;
        bool tog;
        string DropDownCaption;
        void Start()
        {
            if (slider == null) slider = GetComponent<Slider>();
            Cam.depth = InitialDepth;
        }

        public void ChangeFovOrDepth(Slider slider)
        {
            if (gameObject.tag == "FoV")
            {
                Cam.fieldOfView = slider.value;
            }

            if (gameObject.tag == "Depth")
            {
                Cam.depth = slider.value;
            }

            if (gameObject.tag == "NearClipSlider")
            {
                Cam.nearClipPlane = slider.value;
            }

            if (gameObject.tag == "FarClipSlider")
            {
                Cam.farClipPlane = slider.value;
            }

            if (gameObject.tag == "FocalLength" && Cam.usePhysicalProperties == true)
            {
                Cam.focalLength = slider.value;
                double fovdub = Mathf.Rad2Deg * 2.0 * Math.Atan(filmHeight / (2.0 * slider.value));
                FOVslider.value = (float)fovdub;

            }
        }
        public void ChangeToogle(Toggle toggle)
        {
            if (gameObject.tag == "PhysicalProperties")
            {
                tog = toggle.isOn;
                if (tog == true)
                {
                    Cam.usePhysicalProperties = true;
                    IsPhysicalProp = true;

                }
                else if (tog == false)
                {

                    Cam.usePhysicalProperties = false;
                    IsPhysicalProp = false;
                }
            }

            if (gameObject.tag == "Occlusion")
            {
                tog = toggle.isOn;
                Debug.Log(Cam.useOcclusionCulling.ToString());
                if (tog == true & Cam.useOcclusionCulling == false)
                {
                    Cam.useOcclusionCulling = true;
                }
                else if (tog == false & Cam.useOcclusionCulling == true)
                {
                    Cam.useOcclusionCulling = false;
                }
            }
        }
        public void ChangeDropdown(Dropdown dropdown)
        {

            if (gameObject.tag == "FoVAxis")
            {
                // Not useful for the project.
            }

            if (gameObject.tag == "Projection")
            {

                DropDownCaption = EventSystem.current.currentSelectedGameObject.name;
                Debug.Log(DropDownCaption);
                if (DropDownCaption == "Item 0: Perspective")
                {
                    Cam.orthographic = false;
                }
                else if (DropDownCaption == "Item 1: Orthographic")
                {
                    Cam.orthographic = true;
                }
            }
        }
    }
}