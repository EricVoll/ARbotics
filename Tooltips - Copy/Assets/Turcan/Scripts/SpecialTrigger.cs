using UnityEngine;
using UnityEngine.UI;

public class SpecialTrigger : MonoBehaviour
{
    // Start is called before the first frame update
    public Toggle toggleAdvancedFrustum;
    public void ChangeStuff()
    {
         
        //GLDebug.triggerOfDisp = !GLDebug.triggerOfDisp;

        CameraFrustumInGame.triggerOfDispSimple = !CameraFrustumInGame.triggerOfDispSimple;

        if(CameraMesh.triggerOfDispAdvanced)
        {
            toggleAdvancedFrustum.isOn = false;
            CameraMesh.triggerOfDispAdvanced = !CameraMesh.triggerOfDispAdvanced;
        }
    }


//    using UnityEngine;
//using System.Collections;
//using System.Collections.Generic;
//using System;

//public class CameraMesh : MonoBehaviour
//{
//	public Camera Camera;
//	private float dValueX0;
//	private float dValueX3;
//	private float dValueX1;
//	private float dValueX2;
//	private float dValueY0;
//	private float dValueY3;
//	private float dValueY1;
//	private float dValueY2;

//	private float dValueZ0;
//	private float dValueZ3;
//	private float dValueZ1;
//	private float dValueZ2;
//	public int raysToCast = 100;
//	Vector3[] corners2 = new Vector3[4];
//	public static bool triggerOfDispAdvanced = false;

//	private Vector3 dir;
//	private RaycastHit hit;

//	void Update()
//	{
//		TurcanRaySweep();
//	}
//	void TurcanRaySweep()
//	{
//		Vector3[] corners = findcorners(Camera);
//		corners2[0] = transform.InverseTransformPoint(corners[0]);
//		corners2[1] = transform.InverseTransformPoint(corners[1]);
//		corners2[2] = transform.InverseTransformPoint(corners[2]);
//		corners2[3] = transform.InverseTransformPoint(corners[3]);
//		//for (int h = 0; h < 4; h++)
//		//{
//		//	corners2[h] = transform.InverseTransformPoint(corners[h]);
//		//}
//		for (int k = 0; k < 4; k++)
//		{
//			float incrementX = 0;
//			float incrementY = 0;
//			float incrementZ = 0;
//			for (int i = 0; i < raysToCast; i++)
//			{
//				if (k == 0)
//				{
//					dValueX0 = corners[k].x + incrementX;
//					dValueY0 = corners[k].y + incrementY;
//					dValueZ0 = corners[k].z + incrementZ;

//					incrementX -= (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;
//					incrementY -= (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;
//					incrementZ -= (corners[k].z - corners[(k + 1) % 4].z) / raysToCast;

//					dir = new Vector3(dValueX0, corners[k].y, corners[k].z);
//				}
//				if (k == 1)
//				{
//					dValueY1 = corners[k].y + increment;

//					increment -= (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;

//					dir = new Vector3(corners[k].x, dValueY1, corners[k].z);
//				}
//				if (k == 2)
//				{
//					dValueX1 = corners[k].x - increment;

//					increment += (corners[k].x - corners[(k + 1) % 4].x) / raysToCast;

//					dir = new Vector3(dValueX1, corners[k].y, corners[k].z);
//				}
//				if (k == 3)
//				{
//					dValueY1 = corners[k].y - increment;

//					increment += (corners[k].y - corners[(k + 1) % 4].y) / raysToCast;

//					dir = new Vector3(corners[k].x, dValueY1, corners[k].z);
//				}
//				if (triggerOfDispAdvanced)
//				{
//					if (Physics.Raycast(transform.position, dir, out hit))
//					{

//						GLDebug.DrawLine(transform.position, hit.point, Color.green, 0, true);
//						GLDebug.DrawLine(hit.point, dir, Color.red, 0, true);
//					}
//					else
//					{
//						if (i < 3)
//						{
//							GLDebug.DrawLine(transform.position, dir, Color.black, 0, true);
//						}
//						else
//						{
//							GLDebug.DrawLine(transform.position, dir, Color.green, 0, true);
//						}

//						if (i > raysToCast - 4)
//						{
//							GLDebug.DrawLine(transform.position, dir, Color.black, 0, true);
//						}
//						else
//						{
//							GLDebug.DrawLine(transform.position, dir, Color.green, 0, true);
//						}
//					}
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
