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

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of Reading the changes in Dropdown Menus.
/// </summary>

namespace ARRobotInteraction.Sensor
{
    public class ReadDropdown : MonoBehaviour
    {
        Dropdown m_Dropdown;
        public Text m_Text;
        public static string caption;

        public SensorManager SensorManager;

        void Start()
        {
            //Fetch the Dropdown GameObject
            m_Dropdown = GetComponent<Dropdown>();
            //Add listener for when the value of the Dropdown changes, to take action
            m_Dropdown.onValueChanged.AddListener(delegate
            {
                DropdownValueChanged(m_Dropdown);
            });

            //Initialise the Text to say the first value of the Dropdown
            //m_Text.text = "First Value : " + m_Dropdown.value;
        }

        Dictionary<string, SensorType> types = new Dictionary<string, SensorType>()
    {
        { "Item 1: Camera", SensorType.Camera },
        { "Item 2: 2D Lidar", SensorType.Lidar2D },
        { "Item 3: 3D Lidar", SensorType.Lidar3D }
    };

        //Ouput the new value of the Dropdown into Text
        void DropdownValueChanged(Dropdown change)
        {
            caption = EventSystem.current.currentSelectedGameObject.name;

            SensorType typeToCreate = SensorType.Camera;

            if (types.ContainsKey(caption))
                typeToCreate = types[caption];

            SensorManager.CreateSensor(typeToCreate);

            //m_Text.text = "New Value : " + change.value;
            //Debug.Log(EventSystem.current.currentSelectedGameObject.name);

        }
    }
}
