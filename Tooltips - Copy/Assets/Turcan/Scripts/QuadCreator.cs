using UnityEngine;

public class QuadCreator : MonoBehaviour
{
    public float width = 1;
    public float height = 1;

    public void Start()
    {
        MeshRenderer meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = new Material(Shader.Find("Standard"));

        MeshFilter meshFilter = gameObject.AddComponent<MeshFilter>();

        Mesh mesh = new Mesh();

        Vector3[] vertices = new Vector3[4]
        {
            new Vector3(0, 0, 0),
            new Vector3(width, 0, 0),
            new Vector3(0, height, 0),
            new Vector3(width, height, 0)
        };
        mesh.vertices = vertices;

        int[] tris = new int[6]
        {
            // lower left triangle
            0, 2, 1,
            // upper right triangle
            2, 3, 1
        };
        mesh.triangles = tris;

        Vector3[] normals = new Vector3[4]
        {
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward
        };
        mesh.normals = normals;

        Vector2[] uv = new Vector2[4]
        {
            new Vector2(0, 0),
            new Vector2(1, 0),
            new Vector2(0, 1),
            new Vector2(1, 1)
        };
        mesh.uv = uv;

        meshFilter.mesh = mesh;
    }
//    using UnityEngine;
//using System.Collections;
//using System.Collections.Generic;
//using System;

//public class CameraMesh : MonoBehaviour
//{
//	public Camera Camera;
//	private float dValueX1;
//	private float dValueX2;
//	private float dValueY1;
//	private float dValueY2;
//	public int raysToCast;
//	public static int sightRange;
//	public static float angleOfVision; // in radians

//	public Vector3[] vertices;
//	private Vector2[] uvs;
//	public int[] triangles;

//	public Mesh visionConeMesh;
//	public MeshFilter meshFilter;
//	private List<Vector3> pointsT;
//	private float castAngle;
//	private float sinX;
//	private float cosX;
//	private Vector3 dir;
//	private Vector3 temp;
//	private RaycastHit hit;
//	// Start is called before the first frame update
//	void Start()
//	{
//		raysToCast = 200;
//		vertices = new Vector3[raysToCast + 1];
//		uvs = new Vector2[vertices.Length];
//		triangles = new int[(vertices.Length * 3) - 9];

//		// Set up procedural mesh
//		visionConeMesh = new Mesh();
//		visionConeMesh.name = "VisionCone";
//		meshFilter = GetComponent<MeshFilter>();
//		meshFilter.mesh = visionConeMesh;
//	}

//	// Update is called once per frame
//	void Update()
//	{
//		TurcanRaySweep();
//	}
//	void TurcanRaySweep()
//	{
//		pointsT = new List<Vector3>();
//		Vector3[] corners = findcorners(Camera);
//		// angle relative to players'/parents' forward vector
//		//castAngle = -angleOfVision + Mathf.Deg2Rad * transform.eulerAngles.y;

//		/// Sweep rays over the cone of vision ///

//		for (int k = 0; k < 4; k++)
//		{
//			float increment = 0;
//			// cast rays to map out the space in a cone-shaped sweep
//			for (int i = 0; i < raysToCast; i++)
//			{
//				if (k == 0)
//				{
//					dValueX1 = corners[k].x + increment;
//					if (corners[k].x < corners[(i + 1) % 4].x)
//					{
//						increment -= (corners[k].x + corners[(i + 1) % 4].x) / raysToCast;
//					}
//					else
//					{
//						increment += (corners[k].x - corners[(i + 1) % 4].x) / raysToCast;
//					}


//					dir = new Vector3(dValueX1, corners[k].y, corners[k].z);
//				}
//				//if (k == 1)
//				//{
//				//	dValueY1 = corners[k].y + increment;

//				//	increment -= (corners[k].y + corners[(i + 1) % 4].y) / raysToCast;

//				//	dir = new Vector3(corners[k].x, dValueY1, corners[k].z);
//				//}
//				//if (k == 2)
//				//{
//				//	dValueX1 = corners[k].x - increment;

//				//	increment += (corners[k].x + corners[(i + 1) % 4].x) / raysToCast;

//				//	dir = new Vector3(dValueX1, corners[k].y, corners[k].z);
//				//}
//				//if (k == 3)
//				//{
//				//	dValueY1 = corners[k].y - increment;

//				//	increment += (corners[k].y + corners[(i + 1) % 4].y) / raysToCast;

//				//	dir = new Vector3(corners[k].x, dValueY1, corners[k].z);
//				//}



//				if (Physics.Raycast(transform.position, dir, out hit))
//				{

//					GLDebug.DrawLine(transform.position, hit.point, Color.red, 0, true);
//					GLDebug.DrawLine(hit.point, dir, Color.green, 0, true);
//				}
//				else
//				{
//					GLDebug.DrawLine(transform.position, dir, Color.green, 0, true);
//				}

//			}
//		}
//	} // end RaySweep
//	Vector3[] findcorners(Camera cam)
//	{
//		Vector3[] nearCorners = new Vector3[4]; //Approx'd nearplane corners
//		Vector3[] farCorners = new Vector3[4]; //Approx'd farplane corners
//		Plane[] camPlanes = GeometryUtility.CalculateFrustumPlanes(cam); //get planes from matrix

//		Plane temp = camPlanes[1]; camPlanes[1] = camPlanes[2]; camPlanes[2] = temp; //swap [1] and [2] so the order is better for the loop
//		for (int i = 0; i < 4; i++)
//		{
//			nearCorners[i] = Plane3Intersect(camPlanes[4], camPlanes[i], camPlanes[(i + 1) % 4]); //near corners on the created projection matrix
//			farCorners[i] = Plane3Intersect(camPlanes[5], camPlanes[i], camPlanes[(i + 1) % 4]); //far corners on the created projection matrix


//		}
//		return farCorners;
//	}
//	Vector3 Plane3Intersect(Plane p1, Plane p2, Plane p3)
//	{ //get the intersection point of 3 planes
//		return ((-p1.distance * Vector3.Cross(p2.normal, p3.normal)) +
//				(-p2.distance * Vector3.Cross(p3.normal, p1.normal)) +
//				(-p3.distance * Vector3.Cross(p1.normal, p2.normal))) /
//		 (Vector3.Dot(p1.normal, Vector3.Cross(p2.normal, p3.normal)));
//	}
//}

}