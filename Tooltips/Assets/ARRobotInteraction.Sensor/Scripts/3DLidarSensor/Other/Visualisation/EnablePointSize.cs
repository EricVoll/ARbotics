#if UNITY_STANDALONE
#define IMPORT_GLENABLE
#endif
/*
* MIT License
* 
* Copyright (c) 2017 Philip Tibom, Jonathan Jansson, Rickard Laurenius, 
* Tobias Alldén, Martin Chemander, Sherry Davar
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
using System;
using System.Collections;
using System.Runtime.InteropServices;
using ARRobotInteraction.Sensor;

namespace ARRobotInteraction.Sensor
{
    public class EnablePointSize : MonoBehaviour
    {

        const UInt32 GL_VERTEX_PROGRAM_POINT_SIZE = 0x8642;
        const UInt32 GL_POINT_SMOOTH = 0x0B10;

        const string LibGLPath =
#if UNITY_STANDALONE_WIN
        "opengl32.dll";
#elif UNITY_STANDALONE_OSX
    "/System/Library/Frameworks/OpenGL.framework/OpenGL";
#elif UNITY_STANDALONE_LINUX
    "libGL";  // Untested on Linux, this may not be correct
#else
    null;   // OpenGL ES platforms don't require this feature
#endif

#if IMPORT_GLENABLE
        [DllImport(LibGLPath)]
        public static extern void glEnable(UInt32 cap);

        private bool mIsOpenGL;

        void Start()
        {
            mIsOpenGL = SystemInfo.graphicsDeviceVersion.Contains("OpenGL");
        }

        void OnPreRender()
        {
            if (mIsOpenGL)
                glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
            glEnable(GL_POINT_SMOOTH);
        }
#endif

    }
}
