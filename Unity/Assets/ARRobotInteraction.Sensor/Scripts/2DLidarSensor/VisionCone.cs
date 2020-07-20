/*
* MIT License
* 
* Copyright (c) 2020 Eric Vollenweider, Jonas Frey, Raffael Theiler, Turcan Tuna and Benjamin Redahan(Original Contributor, 2015-11-12)
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
using System.Collections.Generic;
using System;
using ARRobotInteraction.Sensor;

namespace ARRobotInteraction.Sensor
{
	public class VisionCone : MonoBehaviour, ISensorDataPublisher
	{

		public int raysToCast;
		public static int sightRange;
		public static float angleOfVision; // in radians

		public Vector3[] vertices;
		private Vector2[] uvs;
		public int[] triangles;

		public Mesh visionConeMesh;
		public MeshFilter meshFilter;

		private float castAngle;
		private float sinX;
		private float cosX;
		private Vector3 dir;
		private Vector3 temp;
		private RaycastHit hit;

		void Start()
		{
			raysToCast = 200;
			vertices = new Vector3[raysToCast + 1];
			uvs = new Vector2[vertices.Length];
			triangles = new int[(vertices.Length * 3) - 9];

			// Set up procedural mesh
			visionConeMesh = new Mesh();
			visionConeMesh.name = "VisionCone";
			meshFilter = GetComponent<MeshFilter>();
			meshFilter.mesh = visionConeMesh;
		}

		// Update is called once per frame
		void Update()
		{
			RaySweep();
		}

		void RaySweep()
		{
			points = new List<Vector3>();

			// angle relative to players'/parents' forward vector
			castAngle = -angleOfVision + Mathf.Deg2Rad * transform.eulerAngles.y;

			/// Sweep rays over the cone of vision ///

			// cast rays to map out the space in a cone-shaped sweep
			for (int i = 0; i < raysToCast; i++)
			{
				sinX = sightRange * Mathf.Sin(castAngle);
				cosX = sightRange * Mathf.Cos(castAngle);

				// Increment in proportion to the size of the cone and the number of rays used to map it
				castAngle += 2 * angleOfVision / raysToCast;

				dir = new Vector3(sinX, 0, cosX);

				//Debug.DrawRay(transform.position, dir, Color.green); // to aid visualization

				if (Physics.Raycast(transform.position, dir, out hit, sightRange))
				{

					temp = transform.InverseTransformPoint(hit.point);
					//temp = hit.point;
					vertices[i] = new Vector3(temp.x, 0.1f, temp.z);
					//Debug.DrawLine (transform.position, hit.point, Color.red); // to aid visualization
					points.Add(hit.point);
				}
				else
				{
					temp = transform.InverseTransformPoint(transform.position + dir);
					//temp = transform.position + dir;
					vertices[i] = new Vector3(temp.x, 0.1f, temp.z);
				}

			} // end raycast loop

			/// Building/Updating the vision cone mesh ///

			// assign the vertices BEFORE dealing with the uvs and triangles
			visionConeMesh.vertices = vertices;

			// created uvs for mesh
			for (int i = 0; i < vertices.Length; i++)
			{
				uvs[i] = new Vector2(vertices[i].x, vertices[i].z);
			} // end uvs loop

			// create triangles for mesh, with each tri ending at the player's location (like pizza slices)
			int x = -1;
			for (int i = 0; i < triangles.Length; i += 3)
			{
				x++;
				triangles[i] = x + 1;
				triangles[i + 1] = x + 2;
				triangles[i + 2] = vertices.Length - 1; // all triangles end at the centre
			}

			visionConeMesh.triangles = triangles;
			visionConeMesh.uv = uvs;

			ExtractPoints();
		} // end RaySweep

		private List<Vector3> points = new List<Vector3>();
		public void ExtractPoints()
		{
			ExtractPointCallBack?.Invoke(points.ToArray());
		}

		public Action<Vector3[]> ExtractPointCallBack { get; set; }

	}
}
