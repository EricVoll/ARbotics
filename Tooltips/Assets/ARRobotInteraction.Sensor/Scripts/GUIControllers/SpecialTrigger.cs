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
/// Is responsible of Controlling the 2 frustum view cone toggles.
/// </summary>

namespace ARRobotInteraction.Sensor
{
    public class SpecialTrigger : MonoBehaviour
    {
        public Toggle toggleAdvancedFrustum;
        public void ChangeStuff()
        {

            CameraFrustumInGame.triggerOfDispSimple = !CameraFrustumInGame.triggerOfDispSimple;

            if (CameraMesh.triggerOfDispAdvanced)
            {
                toggleAdvancedFrustum.isOn = false;
                CameraMesh.triggerOfDispAdvanced = !CameraMesh.triggerOfDispAdvanced;
            }
        }
    }
}
