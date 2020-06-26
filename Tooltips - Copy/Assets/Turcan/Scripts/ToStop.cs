using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToStop : MonoBehaviour
{
    public LidarSensor lidarSensor;
    private bool deneme=false;
    // Start is called before the first frame update
    public void Bla()
    {
        lidarSensor.PauseSensor(deneme);
        deneme = !deneme;
        //blaf = lidarSensor.rotationAnglePerStep;

        //Debug.Log("I am in Bla");
    }
}
