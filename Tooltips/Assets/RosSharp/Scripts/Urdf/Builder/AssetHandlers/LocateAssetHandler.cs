/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

<http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System.IO;
using System;
using UnityEngine;
using System.Xml.Linq;
using System.Globalization;

namespace RosSharp.Urdf.Editor
{
    public static class LocateAssetHandler
    {
        public static void SetRobotName(string name)
        {
            pathPrefix = $"Urdf/Models/{name}/";
        }
        private static string pathPrefix;
        private static string GetPath(string urdfPath)
        {
            if (!urdfPath.StartsWith(@"package://"))
            {
                Debug.LogWarning(urdfPath + " is not a valid URDF package file path. Path should start with \"package://\".");
                return null;
            }

            var path = urdfPath.Substring(10).SetSeparatorChar();

            if (Path.GetExtension(path)?.ToLowerInvariant() == ".stl")
                path = path.Substring(0, path.Length - 3) + "prefab";

            return Path.Combine(pathPrefix, path);
        }
        public static T FindUrdfAsset<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string fileAssetPath = GetPath(urdfFileName);

            ColladaResourceProcessor processor = new ColladaResourceProcessor();
            processor.EvaluateColladaTransformation(fileAssetPath);

            fileAssetPath = System.IO.Path.ChangeExtension(fileAssetPath, null);
            T assetObject = Resources.Load<T>(fileAssetPath);

            if (assetObject is GameObject go)
                processor.ApplyColladaTransformation(go);

            if (assetObject != null)
                return assetObject;

            return null;
        }


        private class InterruptedUrdfImportException : Exception
        {
            public InterruptedUrdfImportException(string message) : base(message)
            {
            }
        }
    }


    public class ColladaResourceProcessor
    {
        private bool isCollada;
        private string orientation;
        private float globalScale;

        public void EvaluateColladaTransformation(string relativeFilePath)
        {
            isCollada = relativeFilePath.EndsWith(".dae");

            if (!isCollada)
                return;

            relativeFilePath = System.IO.Path.ChangeExtension(relativeFilePath, null);

            //globalScale = readGlobalScale(relativeFilePath);

            orientation = readColladaOrientation(relativeFilePath);
        }

        public void ApplyColladaTransformation(GameObject gameObject)
        {
            if (!isCollada)
                return;

            gameObject.transform.SetPositionAndRotation(
                getColladaPositionFix(gameObject.transform.position, orientation),
                Quaternion.Euler(getColladaRotationFix(orientation)) * gameObject.transform.rotation);
        }
        

        private Vector3 getColladaPositionFix(Vector3 position, string orientation)
        {
            switch (orientation)
            {
                case "X_UP": return position; // not tested
                case "Y_UP": return position; // not tested
                case "Z_UP": return new Vector3(-position.z, position.y, -position.x); // tested
                default: return position; // not tested  
            }
        }

        private static Vector3 getColladaRotationFix(string orientation)
        {
            switch (orientation)
            {
                case "X_UP": return new Vector3(-90, 90, 90); // not tested
                case "Y_UP": return new Vector3(-90, 90, 0);  // tested
                case "Z_UP": return new Vector3(0, 90, 0);    // tested
                default: return new Vector3(-90, 90, 0);    // tested                      
            }
        }

        private string readColladaOrientation(string relativeResourcePath)
        {
            try
            {
                System.Xml.Linq.XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
                relativeResourcePath = relativeResourcePath.Replace('/', '\\');
                var fileContent = Resources.Load<TextAsset>(relativeResourcePath);
                Output.Log(relativeResourcePath);
                Output.Log((fileContent != null).ToString());
                XDocument xdoc = XDocument.Parse(fileContent.text);
                var v = xdoc.Element(xmlns + "COLLADA").Element(xmlns + "asset").Element(xmlns + "up_axis").Value;
                Output.Log(v);
                return v;
            }
            catch
            {
                Output.Log("failed");
                return "undefined";
            }
        }

        private float readGlobalScale(string relativeResourcePath)
        {
            try
            {
                System.Xml.Linq.XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
                TextAsset fileContent = Resources.Load(relativeResourcePath) as TextAsset;
                XDocument xdoc = XDocument.Parse(fileContent.text);
                string str = xdoc.Element(xmlns + "COLLADA").Element(xmlns + "asset").Element(xmlns + "unit").Attribute("meter").Value;
                return float.Parse(str, CultureInfo.InvariantCulture.NumberFormat);
            }
            catch
            {
                return 1.0f;
            }
        }
    }

}
