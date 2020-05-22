using UnityEngine;
using System.Collections;
using UnityEditor;

public class CameraFrustumInGame : MonoBehaviour
{
    public Camera Camera;
    public static bool triggerOfDispSimple = false;
    bool[] firstcheck = new bool[4];
    RaycastHit[] hitinf = new RaycastHit[4];
    void Update()
    {
        DrawFrustum(Camera);
    }

    void DrawFrustum(Camera cam)
    {
        if (triggerOfDispSimple)
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

            firstcheck[0] = false;
            firstcheck[1] = false;
            firstcheck[2] = false;
            firstcheck[3] = false;


            for (int i = 0; i < 4; i++)
            {
                //Debug.DrawLine(nearCorners[i], nearCorners[(i + 1) % 4], Color.red, Time.deltaTime, true); //near corners on the created projection matrix
                //Debug.DrawLine(farCorners[i], farCorners[(i + 1) % 4], Color.blue, Time.deltaTime, true); //far corners on the created projection matrix
                //Debug.DrawLine(nearCorners[i], farCorners[i], Color.green, Time.deltaTime, true); //sides of the created projection matrix

                GLDebug.DrawLine(nearCorners[i], nearCorners[(i + 1) % 4], Color.blue, 0, true);
                GLDebug.DrawLine(farCorners[i], farCorners[(i + 1) % 4], Color.blue, 0, true);
                //////////////////////// EASY SOLUTION /////////////////////////////////////

                for (int j = 0; j < 4; j++)
                {
                    firstcheck[j] = Physics.Linecast(nearCorners[j], farCorners[j], out hitinf[j]);
                }

                if (firstcheck[0] && firstcheck[1])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.blue, 0, true);
                }
                if (firstcheck[1] && firstcheck[2])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.blue, 0, true);
                }
                if (firstcheck[2] && firstcheck[3])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.blue, 0, true);
                }
                if (firstcheck[3] && firstcheck[0])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.red, 0, true);
                }
                if (firstcheck[0] && firstcheck[1] && firstcheck[2])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.blue, 0, true);
                }
                if (firstcheck[1] && firstcheck[2] && firstcheck[3])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.blue, 0, true);
                }
                if (firstcheck[2] && firstcheck[3] && firstcheck[0])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.red, 0, true);
                }
                if (firstcheck[3] && firstcheck[0] && firstcheck[1])
                {
                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.red, 0, true);

                }
                if (firstcheck[3] && firstcheck[0] && firstcheck[1] && firstcheck[2])
                {

                    GLDebug.DrawLine(farCorners[0], farCorners[(1) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[1], farCorners[(2) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[2], farCorners[(3) % 4], Color.red, 0, true);
                    GLDebug.DrawLine(farCorners[3], farCorners[(4) % 4], Color.red, 0, true);
                }
                if (!firstcheck[3] && !firstcheck[0] && !firstcheck[1] && !firstcheck[2])
                {
                    GLDebug.DrawLine(nearCorners[i], nearCorners[(i + 1) % 4], Color.blue, 0, true);
                    GLDebug.DrawLine(farCorners[i], farCorners[(i + 1) % 4], Color.blue, 0, true);
                }
                /////////////////////////  EASY SOLUTION END ////////////////////////

                if (firstcheck[i])
                {
                    GLDebug.DrawLine(nearCorners[i], hitinf[i].point, Color.blue, 0, true);
                    GLDebug.DrawLine(hitinf[i].point, farCorners[i], Color.red, 0, true);

                }
                else
                {
                    GLDebug.DrawLine(nearCorners[i], farCorners[i], Color.blue, 0, true);
                }

            }
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