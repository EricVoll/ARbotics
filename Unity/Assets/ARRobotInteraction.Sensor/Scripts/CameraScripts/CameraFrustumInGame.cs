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
using ARRobotInteraction.Sensor;
/// <summary>
/// Is responsible of controlling Frustum Visualization for the Simple Frustum
/// </summary>
namespace ARRobotInteraction.Sensor
{

    public class CameraFrustumInGame : MonoBehaviour
    {
        public DrawScript DrawScript;
        public Camera Camera;
        bool onetime = false;
        public static bool triggerOfDispSimple = false;
        bool[] firstcheck = new bool[4];
        RaycastHit[] hitinf = new RaycastHit[4];
        void Update()
        {
            if (!onetime)
            {
                DrawFrustum(Camera);
                onetime = true;
            }
            DrawFrustum(Camera);
        }

        void DrawFrustum(Camera cam)
        {
            if (triggerOfDispSimple)
            {
                Vector3[] nearCorners = new Vector3[4]; 
                Vector3[] farCorners = new Vector3[4]; 
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

                    DrawScript.DrawLineTRC(nearCorners[i], nearCorners[(i + 1) % 4], 0.02f, "blue");
                    DrawScript.DrawLineTRC(farCorners[i], farCorners[(i + 1) % 4], 0.02f, "blue");

                    //GLDebug.DrawLine(nearCorners[i], nearCorners[(i + 1) % 4], Color.blue, 0, true);
                    //GLDebug.DrawLine(farCorners[i], farCorners[(i + 1) % 4], Color.blue, 0, true);
                    //////////////////////// EASY SOLUTION /////////////////////////////////////

                    for (int j = 0; j < 4; j++)
                    {
                        firstcheck[j] = Physics.Linecast(nearCorners[j], farCorners[j], out hitinf[j]);
                    }

                    if (firstcheck[0] && firstcheck[1])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "blue");

                    }
                    if (firstcheck[1] && firstcheck[2])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "blue");

                    }
                    if (firstcheck[2] && firstcheck[3])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "blue");

                    }
                    if (firstcheck[3] && firstcheck[0])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "red");

                    }
                    if (firstcheck[0] && firstcheck[1] && firstcheck[2])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "blue");

                    }
                    if (firstcheck[1] && firstcheck[2] && firstcheck[3])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "blue");

                    }
                    if (firstcheck[2] && firstcheck[3] && firstcheck[0])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "red");

                    }
                    if (firstcheck[3] && firstcheck[0] && firstcheck[1])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "red");

                    }
                    if (firstcheck[3] && firstcheck[0] && firstcheck[1] && firstcheck[2])
                    {
                        DrawScript.DrawLineTRC(farCorners[0], farCorners[(1) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[1], farCorners[(2) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[2], farCorners[(3) % 4], 0.02f, "red");
                        DrawScript.DrawLineTRC(farCorners[3], farCorners[(4) % 4], 0.02f, "red");

                    }
                    if (!firstcheck[3] && !firstcheck[0] && !firstcheck[1] && !firstcheck[2])
                    {
                        DrawScript.DrawLineTRC(nearCorners[i], nearCorners[(i + 1) % 4], 0.02f, "blue");
                        DrawScript.DrawLineTRC(farCorners[i], farCorners[(1) % 4], 0.02f, "blue");
                    }
                    /////////////////////////  EASY SOLUTION END ////////////////////////

                    if (firstcheck[i])
                    {
                        DrawScript.DrawLineTRC(nearCorners[i], hitinf[i].point, 0.02f, "blue");
                        DrawScript.DrawLineTRC(hitinf[i].point, farCorners[i], 0.02f, "red");

                    }
                    else
                    {
                        DrawScript.DrawLineTRC(nearCorners[i], farCorners[i], 0.02f, "blue");
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
}