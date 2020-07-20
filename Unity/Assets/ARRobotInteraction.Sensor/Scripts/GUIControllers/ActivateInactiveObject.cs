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
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of Activating a Inactive Object
/// </summary>
namespace ARRobotInteraction.Sensor
{
    public class ActivateInactiveObject : MonoBehaviour
    {
        public GameObject ToBeActivatedb;
        public Toggle myToggle;
        // Start is called before the first frame update
        public void ActivateInactiveObj()
        {
            if (CameraPropertiesChange.IsPhysicalProp == true)
            {
                ToBeActivatedb.SetActive(true);
            }
            else if (CameraPropertiesChange.IsPhysicalProp == false)
            {
                ToBeActivatedb.SetActive(false);
            }

            if (ToBeActivatedb.tag == "Rays")
            {
                if (myToggle.isOn)
                { ToBeActivatedb.SetActive(true); }
                if (!myToggle.isOn)
                {
                    ToBeActivatedb.SetActive(false);
                }
            }
        }
    }
}
