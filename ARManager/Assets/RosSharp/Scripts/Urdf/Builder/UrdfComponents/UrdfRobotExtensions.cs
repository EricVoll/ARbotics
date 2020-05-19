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

using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfRobotExtensions
    {
        public static void Create()
        {
            GameObject robotGameObject = new GameObject("Robot");
            robotGameObject.AddComponent<UrdfRobot>();

            UrdfPlugins.Synchronize(robotGameObject.transform);

            UrdfLink urdfLink = UrdfLinkExtensions.Synchronize(robotGameObject.transform);
            urdfLink.name = "base_link";
            urdfLink.IsBaseLink = true;
        }

        #region Import

        /// <summary>
        /// Generates the Robot-GameObject from the urdf content in the specified file and returns the GameObject created
        /// </summary>
        /// <param name="filename"></param>
        /// <param name="containerObject"></param>
        /// <returns></returns>
        public static GameObject Create(string filename)
        {
            Robot robot = new Robot(filename);

            return Create(robot);
        }

        public static GameObject Create(Robot robot)
        {
            if (string.IsNullOrEmpty(robot.filename))
            {
                Debug.LogError("Filename on Robot must not be empty");
                return null;
            }

            if (!UrdfAssetPathHandler.IsValidAssetPath(robot.filename))
            {
                Debug.LogError("URDF file and ressources must be placed in Assets Folder:\n" + Application.dataPath);
                return null;
            }

            GameObject robotGameObject = new GameObject($"RobotRoot_{robot.name}");

            robotGameObject.AddComponent<UrdfRobot>();

            UrdfAssetPathHandler.SetPackageRoot(Path.GetDirectoryName(robot.filename));
            UrdfMaterial.InitializeRobotMaterials(robot);
            UrdfPlugins.Synchronize(robotGameObject.transform, robot.plugins);

            UrdfLinkExtensions.Synchronize(robotGameObject.transform, robot.root);

            GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
            Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
            Selection.activeObject = robotGameObject;

            return robotGameObject;
        }

        #endregion
        
    }
}
