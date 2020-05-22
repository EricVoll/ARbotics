using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FadeScript : MonoBehaviour
{
    [SerializeField] private GameObject _sphere;
    [SerializeField] private GameObject _cube;

    private float myProximity = 5;


    void Update()
    {
        float absoluteDistance = Mathf.Abs(Vector3.Distance(_cube.transform.position, _sphere.transform.position));

        if (absoluteDistance > myProximity)
            GetComponent<Renderer>().material.color = new Color(1, 1, 1, 0);
        else if (absoluteDistance < myProximity && absoluteDistance > 0)
        {
            float alpha = 1 / absoluteDistance / myProximity; // Get the inverse as it gets closer
            GetComponent<Renderer>().material.color = new Color(1, 1, 1, alpha);
        }
        else
            GetComponent<Renderer>().material.color = new Color(1, 1, 1, 1);

        Vector3 _cubePos = _cube.transform.position;
        _cube.transform.position = new Vector3(_cubePos.x + Input.GetAxis("Horizontal"), _cubePos.y, _cubePos.z);
    }
}