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
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of Making the Sensors a child of the Robot to move together.
/// </summary>

namespace ARRobotInteraction.Sensor
{
    public class ParentingScript : MonoBehaviour
    {
        public static bool hasParent = false;
        public GameObject Child;
        private GameObject Parent2;

        public void OnTriggerEnter(Collider other)
        {
            if (hasParent == false)
            {
                Parent2 = FindClosestRobot();
                Child.transform.SetParent(Parent2.transform, true);
            }

            hasParent = true;

        }
        public GameObject FindClosestRobot()
        {
            GameObject[] gos;
            gos = GameObject.FindGameObjectsWithTag("Robot");
            GameObject closest = null;
            float distance = Mathf.Infinity;
            Vector3 position = transform.position;
            foreach (GameObject go in gos)
            {
                Vector3 diff = go.transform.position - position;
                float curDistance = diff.sqrMagnitude;
                if (curDistance < distance)
                {
                    closest = go;
                    distance = curDistance;
                }
            }
            return closest;
        }
    }
}
