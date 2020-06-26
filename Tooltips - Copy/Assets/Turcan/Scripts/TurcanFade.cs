using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TurcanFade : MonoBehaviour
{
    //Color far =
    public GameObject obj1;
    public GameObject obj2;

    Color near = Color.red;
    Color far = Color.green;
    const float MAX_DISTANCE = 2;
        public void Start()
    {
        far=obj1.GetComponent<Renderer>().material.color;
    }
        void Update()
    {
        
        //Get distance between those two Objects
        float distanceApart = getSqrDistance(obj1.transform.position, obj2.transform.position);
        //UnityEngine.Debug.Log(getSqrDistance(obj1.transform.position, obj2.transform.position));

        //Convert 0 and 200 distance range to 0f and 1f range
        float lerp = mapValue(distanceApart, 0, MAX_DISTANCE, 0f, 1f);

        //Lerp Color between near and far color
        Color lerpColor = Color.Lerp(near, far, lerp);
        obj1.GetComponent<Renderer>().material.color = lerpColor;
    }

    public float getSqrDistance(Vector3 v1, Vector3 v2)
    {
        return (v1 - v2).sqrMagnitude;
    }

    float mapValue(float mainValue, float inValueMin, float inValueMax, float outValueMin, float outValueMax)
    {
        return (mainValue - inValueMin) * (outValueMax - outValueMin) / (inValueMax - inValueMin) + outValueMin;
    }
}
