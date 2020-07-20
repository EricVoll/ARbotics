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
using UnityEngine;
using UnityEngine.UI;
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of controlling Frustum Visualization for the Advanced Frustum
/// </summary>
namespace ARRobotInteraction.Sensor
{
	public class CameraMesh : MonoBehaviour
	{
		public DrawScript DrawScript;
		public Camera Camera;
		private float dValueX0;
		private float dValueX3;
		private float dValueX1;
		private float dValueX2;
		private float dValueY0;
		private float dValueY3;
		private float dValueY1;
		private float dValueY2;
		public Slider slider;
		private float dValueZ0;
		private float dValueZ3;
		private float dValueZ1;
		private float dValueZ2;
		public int raysToCast;
		public static bool triggerOfDispAdvanced = false;

		private Vector3 dir;
		private RaycastHit hit;

		void Update()
		{
			TurcanRaySweep();
		}
		void TurcanRaySweep()
		{
			if (triggerOfDispAdvanced)
			{
				Vector3[] corners = findcorners(Camera);

				for (int k = 0; k < 4; k++)
				{
					float incrementX = 0;
					float incrementY = 0;
					float incrementZ = 0;
					for (int i = 0; i < raysToCast; i++)
					{
						if (k == 0)
						{
							dValueX0 = corners[k].x + incrementX;
							dValueY0 = corners[k].y + incrementY;
							dValueZ0 = corners[k].z + incrementZ;

							incrementX -= (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;
							incrementY -= (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;
							incrementZ -= (corners[k].z - corners[(k + 1) % 4].z) / raysToCast;

							dir = new Vector3(dValueX0, dValueY0, dValueZ0);
						}
						if (k == 1)
						{
							dValueX1 = corners[k].x + incrementX;
							dValueY1 = corners[k].y + incrementY;
							dValueZ1 = corners[k].z + incrementZ;

							incrementX -= (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;
							incrementY -= (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;
							incrementZ -= (corners[k].z - corners[(k + 1) % 4].z) / raysToCast;

							dir = new Vector3(dValueX1, dValueY1, dValueZ1);
						}
						if (k == 2)
						{
							dValueX2 = corners[k].x - incrementX;
							dValueY2 = corners[k].y - incrementY;
							dValueZ2 = corners[k].z - incrementZ;

							incrementX += (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;
							incrementY += (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;
							incrementZ += (corners[k].z - corners[(k + 1) % 4].z) / raysToCast;

							dir = new Vector3(dValueX2, dValueY2, dValueZ2);
						}
						if (k == 3)
						{
							dValueX3 = corners[k].x - incrementX;
							dValueY3 = corners[k].y - incrementY;
							dValueZ3 = corners[k].z - incrementZ;

							incrementX += (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;
							incrementY += (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;
							incrementZ += (corners[k].z - corners[(k + 1) % 4].z) / raysToCast;

							dir = new Vector3(dValueX3, dValueY3, dValueZ3);
						}

						if (Physics.Linecast(Camera.transform.position, dir, out hit))
						{
							DrawScript.DrawLineTRC(Camera.transform.position, hit.point, 0.02f, "green");
							DrawScript.DrawLineTRC(hit.point, dir, 0.02f, "red");
						}
						else
						{
							if (i == 0)
							{
								DrawScript.DrawLineTRC(Camera.transform.position, dir, 0.02f, "blue");
							}
							else
							{
								DrawScript.DrawLineTRC(Camera.transform.position, dir, 0.02f, "green");
							}
						}
					}
				}
			}
		}

		public void changeRayNum(Slider slider)
		{
			raysToCast = (int)slider.value;
		}
		Vector3[] findcorners(Camera cam)
		{
			Vector3[] nearCorners = new Vector3[4]; //Approx'd nearplane corners
			Vector3[] farCorners = new Vector3[4]; //Approx'd farplane corners
			Plane[] camPlanes = GeometryUtility.CalculateFrustumPlanes(cam); //get planes from matrix

			Plane temp = camPlanes[1]; camPlanes[1] = camPlanes[2]; camPlanes[2] = temp; //swap [1] and [2] so the order is better for the loop
			for (int i = 0; i < 4; i++)
			{
				nearCorners[i] = Plane3Intersect(camPlanes[4], camPlanes[i], camPlanes[(i + 1) % 4]); //near corners on the created projection matrix
				farCorners[i] = Plane3Intersect(camPlanes[5], camPlanes[i], camPlanes[(i + 1) % 4]); //far corners on the created projection matrix		
			}
			return farCorners;
		}
		Vector3 Plane3Intersect(Plane p1, Plane p2, Plane p3)
		{ //get the intersection point of 3 planes
			return ((-p1.distance * Vector3.Cross(p2.normal, p3.normal)) +
					(-p2.distance * Vector3.Cross(p3.normal, p1.normal)) +
					(-p3.distance * Vector3.Cross(p1.normal, p2.normal))) /
			 (Vector3.Dot(p1.normal, Vector3.Cross(p2.normal, p3.normal)));
		}
	}
}
