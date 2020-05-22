/// <summary>
/// 2015-11-12 Benjamin Redahan
/// Procedural Mesh Cone of Vision
/// </summary>

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

public class VisionCone : MonoBehaviour, ISensorDataPublisher {

	public int 			raysToCast;
	public static int 	sightRange;
	public static float angleOfVision; // in radians

	public Vector3[] 	vertices;
	private Vector2[] 	uvs;
	public int[] 		triangles;

	public Mesh 		visionConeMesh;
	public MeshFilter	meshFilter;

	private float 		castAngle;
	private float 		sinX;
	private float		cosX;
	private Vector3		dir;
	private Vector3		temp;
	private RaycastHit 	hit;

	// Use this for initialization
	void Start () 
	{
		raysToCast = 200;
		vertices = new Vector3[raysToCast + 1];
		uvs = new Vector2[vertices.Length];
		triangles = new int[(vertices.Length * 3) - 9];

		// Set up procedural mesh
		visionConeMesh = new Mesh();
		visionConeMesh.name = "VisionCone";
		meshFilter = GetComponent<MeshFilter> ();
		meshFilter.mesh = visionConeMesh;
	}
	
	// Update is called once per frame
	void Update () 
	{
		RaySweep ();
	}

	void RaySweep()
	{
		points = new List<Vector3>();

		// angle relative to players'/parents' forward vector
		castAngle = -angleOfVision + Mathf.Deg2Rad*transform.eulerAngles.y; 

		/// Sweep rays over the cone of vision ///

		// cast rays to map out the space in a cone-shaped sweep
		for(int i = 0; i < raysToCast; i++)
		{
			sinX = sightRange * Mathf.Sin(castAngle);
			cosX = sightRange * Mathf.Cos(castAngle);

			// Increment in proportion to the size of the cone and the number of rays used to map it
			castAngle += 2*angleOfVision/raysToCast;
			
			dir = new Vector3(sinX,0,cosX);

			//Debug.DrawRay(transform.position, dir, Color.green); // to aid visualization
			
			if(Physics.Raycast(transform.position, dir, out hit, sightRange))
			{

     				temp = transform.InverseTransformPoint(hit.point);
				//temp = hit.point;
				vertices[i] = new Vector3(temp.x,0.1f,temp.z);
				//Debug.DrawLine (transform.position, hit.point, Color.red); // to aid visualization
				points.Add(hit.point);
			} 
			else
			{
				temp = transform.InverseTransformPoint(transform.position + dir);
				//temp = transform.position + dir;
				vertices[i] = new Vector3(temp.x,0.1f,temp.z);
			}
			
		} // end raycast loop

		/// Building/Updating the vision cone mesh ///

		// assign the vertices BEFORE dealing with the uvs and triangles
		visionConeMesh.vertices = vertices;

		// created uvs for mesh
		for(int i = 0; i < vertices.Length; i++)
		{
			uvs[i] = new Vector2(vertices[i].x, vertices[i].z);
		} // end uvs loop

		// create triangles for mesh, with each tri ending at the player's location (like pizza slices)
		int x = -1;
		for(int i = 0; i < triangles.Length; i+=3){
			x++;
			triangles[i] = x+1;
			triangles[i+1] = x+2;
			triangles[i+2] = vertices.Length-1; // all triangles end at the centre
		}

		visionConeMesh.triangles = triangles;
		visionConeMesh.uv = uvs;

		//visionConeMesh.RecalculateNormals (); // not sure if this is necessary anymore
		ExtractPoints();
	} // end RaySweep

	private List<Vector3> points = new List<Vector3>();
	public void ExtractPoints()
	{
		ExtractPointCallBack?.Invoke(points.ToArray());
	}

	public Action<Vector3[]> ExtractPointCallBack { get; set; }

}
