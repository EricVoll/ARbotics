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

    }

    public void Synchronize(Robot robot, GameObject rootObject)
    {
        GameObject robotGameObject = rootObject;

        robotGameObject.AddComponentIfNotExists<UrdfRobot>();

        UrdfAssetPathHandler.SetPackageRoot(Path.GetDirectoryName(robot.filename));
        UrdfMaterial.InitializeRobotMaterials(robot);
        UrdfPlugins.Synchronize(robotGameObject.transform, robot.plugins);

        UrdfLinkExtensions.Synchronize(robotGameObject.transform, robot.root);
        //GameObjectUtility.SetParentAndAlign(robotGameObject, Selection.activeObject as GameObject);
        //Undo.RegisterCreatedObjectUndo(robotGameObject, "Create " + robotGameObject.name);
        //Selection.activeObject = robotGameObject;
    }


}





public interface ISensorManager
{
    GameObject GetSensor();
}

public class SensorMAnager : ISensorManager
{
    public GameObject GetSensor()
    {
        return new GameObject("Cool");
    }
}

public class SecondClass : ISensorManager
{
    public GameObject GetSensor()
    {
        return new GameObject("SuperCool");
    }
}


public class MainClass
{
    public void Main()
    {
        ISensorManager obj = new SensorMAnager();

        for (int i = 0; i < 123; i++)
        {
            obj.GetSensor();
        }
    }
}












