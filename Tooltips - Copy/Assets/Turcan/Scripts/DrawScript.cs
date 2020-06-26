using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawScript : MonoBehaviour
{
    //public LineRenderer lineRenderer;
    public Material Redmaterial;
    public Material Greenmaterial;
    public Material Bluematerial;
    public GameObject lineDrawerPrefab;

    void Start()
    {
        //Redmaterial = (Material)Resources.Load("Materials/Environment/Cube Red.mat", typeof(Material)) as Material;
        //Greenmaterial = (Material)Resources.Load("Cube Green", typeof(Material));
        //Bluematerial = (Material)Resources.Load("Cube Blue", typeof(Material));

        //lineRenderer = GetComponent<LineRenderer>();
        //Vector3[] positions = new Vector3[3] { new Vector3(0, 0, 0), new Vector3(-1, 1, 0), new Vector3(1, 1, 0) };
        ////DrawTriangle(positions, 0.02f, 0.02f);
        //DrawLineTRC(new Vector3(0, 0, 0), new Vector3(0, 1, 0), 5, "red");
        //DrawLineTRC(new Vector3(0, 0.1f, 0.1f), new Vector3(0, 2, 0), 2, "blue");
    }

    //void DrawTriangle(Vector3[] vertexPositions, float startWidth, float endWidth)
    //{
    //    lineRenderer.startWidth = startWidth;
    //    lineRenderer.endWidth = endWidth;
    //    lineRenderer.loop = true;
    //    lineRenderer.positionCount = 3;
    //    lineRenderer.SetPositions(vertexPositions);
    //}
    public void DrawLineTRC(Vector3 start, Vector3 end, float duration = 0.2f, string ColorSpace="green")
    {
        //GameObject myLine = new GameObject();
        //myLine.transform.position = start;
        //myLine.AddComponent<LineRenderer>();
        GameObject lineDrawer = Instantiate(lineDrawerPrefab);
        LineRenderer lr = lineDrawer.GetComponent<LineRenderer>();

        //lr.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
        //lineRenderer.startColor = color;
        //lineRenderer.endColor = color;
        lr.startWidth = 0.02f;
        //lineRenderer.material.color = color;
        //materials = lineRenderer.material;
        if(ColorSpace=="red")
        {
            lr.material = Redmaterial;
        }
        else if(ColorSpace == "green")
        {
            lr.material = Greenmaterial;
        }
        else if (ColorSpace == "blue")
        {
            lr.material = Bluematerial;
        }
        lr.SetPosition(0, start);
        lr.SetPosition(1, end);
        GameObject.Destroy(lineDrawer, duration);
    }
}
