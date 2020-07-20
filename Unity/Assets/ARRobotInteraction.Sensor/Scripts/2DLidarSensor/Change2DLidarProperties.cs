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
using ARRobotInteraction.Sensor;

/// <summary>
/// Is responsible of controlling 2DLidar Menu Properties.
/// </summary>
namespace ARRobotInteraction.Sensor
{
    public class Change2DLidarProperties : MonoBehaviour
    {
        // Start is called before the first frame update
        public Slider slider;
        void Start()
        {
            if (slider == null) slider = GetComponent<Slider>();
            if (gameObject.tag == "AngleOfVision")
            {
                VisionCone.angleOfVision = slider.value * (Mathf.PI / 180.0f);
            }


            if (gameObject.tag == "SightDepth")
            {
                VisionCone.sightRange = (int)slider.value;
            }
        }
        public void ChangeVisOrDepth(Slider slider)
        {
            //Cam = gameObject.GetComponent<Camera>();
            //Set up the maximum and minimum values the Slider can return (you can change these)
            //max = 150.0f;
            //min = 20.0f;
            //Debug.Log("ChangeFov");
            if (gameObject.tag == "AngleOfVision")
            {
                VisionCone.angleOfVision = slider.value * (Mathf.PI / 180.0f);
            }
            if (gameObject.tag == "SightDepth")
            {
                VisionCone.sightRange = (int)slider.value;
            }
        }
    }
}
