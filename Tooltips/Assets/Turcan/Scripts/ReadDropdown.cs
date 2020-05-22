using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

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
        m_Dropdown.onValueChanged.AddListener(delegate {
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
