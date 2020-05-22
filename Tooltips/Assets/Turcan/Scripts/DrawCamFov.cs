using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawCamFov : MonoBehaviour
{

    public Camera cam;

    GameObject[] Triangles;

    private void Start()
    {
        Triangles = new GameObject[4];
        for (int i = 0; i < 4; i++)
        {
            GameObject child = new GameObject();
            MeshFilter mf = child.AddComponent<MeshFilter>();
            child.transform.parent = cam.transform;
            Triangles[i] = child;

            Vector3[] vertices = new Vector3[]
            {
                Vector3.zero,
                Vector3.zero,
                Vector3.zero
            };
            int[] triangles = new int[]
            {
             2,1,0
            };

            mf.mesh = new Mesh();
            mf.mesh.vertices = vertices;
            mf.mesh.triangles = triangles;

            child.AddComponent<MeshRenderer>();
        }
    }

    void Update()
    {
        DrawFrustum(cam);
    }

    void DrawFrustum(Camera cam)
    {
        Vector3[] frustumCorners = new Vector3[4];
        cam.CalculateFrustumCorners(new Rect(0, 0, 1, 1), cam.farClipPlane, Camera.MonoOrStereoscopicEye.Mono, frustumCorners);

        for (int i = 0; i < 4; i++)
        {
            var worldSpaceCorner = cam.transform.TransformVector(frustumCorners[i]);
            Debug.DrawRay(cam.transform.position, worldSpaceCorner, Color.blue);

            Triangles[i].GetComponent<MeshFilter>().mesh.vertices = new[]
             {
                     cam.transform.position,
                     frustumCorners[i],
                     frustumCorners[(i + 1) % 4]

            };
        }
    }

    Vector3 Plane3Intersect(Plane p1, Plane p2, Plane p3)
    { //get the intersection point of 3 planes
        return ((-p1.distance * Vector3.Cross(p2.normal, p3.normal)) +
                (-p2.distance * Vector3.Cross(p3.normal, p1.normal)) +
                (-p3.distance * Vector3.Cross(p1.normal, p2.normal))) /
         (Vector3.Dot(p1.normal, Vector3.Cross(p2.normal, p3.normal)));
    }
}