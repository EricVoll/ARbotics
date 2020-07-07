using ARRobotInteraction.Data;
using RosSharp.Urdf;
using RosSharp.Urdf.Editor;
using UnityEngine;

namespace ARRobotInteraction.Base
{
    public class RobotBuilder
    {
        public RobotBuilder()
        {
            AttachedDataSynchronizer.Instance.ShouldCreate = AttachablesManager.Instance.ShouldCreate;
            AttachedDataSynchronizer.Instance.CreateCallBack = AttachablesManager.Instance.Subscribe;
            AttachedDataSynchronizer.Instance.DestroyCallBack = AttachablesManager.Instance.Unsubscribe;
        }

        /// <summary>
        /// Synchronizes the <paramref name="rootObject"/> with the <paramref name="robot"/>
        /// </summary>
        /// <param name="robot"></param>
        /// <param name="rootObject"></param>
        public void Synchronize(Robot robot, GameObject rootObject)
        {
            LocateAssetHandler.SetRobotName(robot.name);

            GameObject robotGameObject = rootObject;

            robotGameObject.AddComponentIfNotExists<UrdfRobot>();

            UrdfPlugins.Synchronize(robotGameObject.transform, robot.plugins);

            UrdfLinkExtensions.Synchronize(robotGameObject.transform, robot.root);
        
            //These tags are needed for the Camera/Sensors to interact with the gameobject
            foreach(var child in rootObject.GetComponentsInChildren<Transform>()) {
                child.gameObject.tag = "Robot";
            }
        }
    }
}