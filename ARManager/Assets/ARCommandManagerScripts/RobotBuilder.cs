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

//using RobotVisualizer;
using RosSharp.Urdf;
using RosSharp.Urdf.Editor;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

public class RobotBuilder
{
    public RobotBuilder()
    {
        //AttachedDataSynchronizer.Instance.CreateCallBack = AttachablesManager.Instance.Subscribe;
    }

    public void Synchronize(Robot robot, GameObject rootObject, string assetRoot)
    {
        GameObject robotGameObject = rootObject;

        robotGameObject.AddComponentIfNotExists<UrdfRobot>();

        UrdfAssetPathHandler.SetPackageRoot(assetRoot);
        UrdfMaterial.InitializeRobotMaterials(robot);
        UrdfPlugins.Synchronize(robotGameObject.transform, robot.plugins);

        UrdfLinkExtensions.Synchronize(robotGameObject.transform, robot.root);
    }

}