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
        public static T FindUrdfAsset<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string fileAssetPath = UrdfAssetPathHandler.GetRelativeAssetPathFromUrdfPath(urdfFileName);
            fileAssetPath = fileAssetPath.Substring(7, fileAssetPath.Length - 7);

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
            relativeFilePath = relativeFilePath.Replace('/', '\\');
            
            string absoluteFilePath = getAbsolutePath(relativeFilePath);

            isCollada = absoluteFilePath.EndsWith(".dae");

            if (!isCollada)
                return;

            globalScale = readGlobalScale(absoluteFilePath);

            orientation = readColladaOrientation(absoluteFilePath);
        }

        public void ApplyColladaTransformation(GameObject gameObject)
        {
            if (!isCollada)
                return;

            gameObject.transform.SetPositionAndRotation(
                getColladaPositionFix(gameObject.transform.position, orientation),
                Quaternion.Euler(getColladaRotationFix(orientation)) * gameObject.transform.rotation);
        }

        private static string getAbsolutePath(string relativeAssetPath)
        {
            return Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets", "Resources", relativeAssetPath);
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
                case "Z_UP": return new Vector3(0, -90, 0);    // tested
                default: return new Vector3(-90, 90, 0);    // tested                      
            }
        }

        private string readColladaOrientation(string absolutePath)
        {
            try
            {
                System.Xml.Linq.XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
                XDocument xdoc = XDocument.Load(absolutePath);
                return xdoc.Element(xmlns + "COLLADA").Element(xmlns + "asset").Element(xmlns + "up_axis").Value;
            }
            catch
            {
                return "undefined";
            }
        }

        private float readGlobalScale(string absolutePath)
        {
            try
            {
                System.Xml.Linq.XNamespace xmlns = "http://www.collada.org/2005/11/COLLADASchema";
                XDocument xdoc = XDocument.Load(absolutePath);
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
