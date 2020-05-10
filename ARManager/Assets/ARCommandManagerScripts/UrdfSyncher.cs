using RosSharp.RosBridgeClient;
using RosSharp.Urdf;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

public class UrdfSyncher
{

    #region Static Inits
    private static AttachableComponentFactory<IAttachableComponent> tooltipFactory;

    /// <summary>
    /// Initializes all static classes needed for the urdfSyncher to work
    /// </summary>
    public static void InitializeUrdfSyncher()
    {
        //Attach tooltip factory to the Robot Parser
        tooltipFactory = new AttachableComponentFactory<IAttachableComponent>("tooltip")
        {
            Constructor = () => new RosSharp.Urdf.Attachables.AttachedDataValue(),
        };

        Robot.attachableComponentFactories.Add(tooltipFactory);
    }
    #endregion

    /// <summary>
    /// 
    /// </summary>
    /// <param name="sceneContainerObject">The gameobject which should be used to contain this robot</param>
    public UrdfSyncher(GameObject sceneContainerObject, string robotName, int recreateObjectAfterSynchronizations = -1)
    {
        SceneContainerGameObject = sceneContainerObject;
        synchronizationCounterMax = recreateObjectAfterSynchronizations;
        RobotName = robotName;
    }

    /// <summary>
    /// The Root gameobject which will be used to spawn all robots. This will be the parent
    /// </summary>
    public GameObject SceneContainerGameObject;

    public string RobotName;

    /// <summary>
    /// The root gameobject of this robot
    /// </summary>
    private GameObject RobotRootObject;

    private bool hasUrdfAssetsImported = false;

    private string assetsRootDirectoryName;

    private int synchronizationCounterWithoutFullRegeneration = 0;
    private int synchronizationCounterMax = -1;

    private bool debugMode = false;

    /// <summary>
    /// Compares the current devices in the scene with the urdf file and performs adjustments
    /// </summary>
    /// <param name="urdf"></param>
    public void SynchUrdf(string urdf)
    {
        ReportSynch();

        Robot robot = Robot.FromContent(urdf);

        if (!debugMode)
        {
            if (!hasUrdfAssetsImported)
            {
                ImportInitialUrdfModel(urdf, robot);
                return;
            }
        }
        else
        {

            if (RobotRootObject == null)
            {
                RobotRootObject = new GameObject("MyCoolRoot");
                RobotRootObject.transform.SetParent(SceneContainerGameObject.transform);
            }
            assetsRootDirectoryName = @"C:\Users\ericv\Documents\ETH\ETH 2020\3D Vision\3D_Vision_AR_RobotVis\ARManager\Assets\Urdf\Models\turtlebot\Turtlebot2\robot_description.urdf";

            robot.filename = assetsRootDirectoryName;
        }


        //we do not return here since we want to apply any possible changes that were passed in the last urdf update.
        //the imported downloads the urdf from the file_server so it might be an old version.
        //If there are no changes to be made - even better.

        RobotBuilder builder = new RobotBuilder();
        builder.Synchronize(robot, RobotRootObject, assetsRootDirectoryName);

        synchronizationCounterWithoutFullRegeneration++;
    }

    /// <summary>
    /// Imports the URDF Model initially to download all textures, assets and store them correctly etc.
    /// </summary>
    /// <param name="robot"></param>
    private void ImportInitialUrdfModel(string urdf, Robot robot)
    {
        hasUrdfAssetsImported = false;

        var protocol = RosConnector.Protocols.WebSocketNET;
        RosSharp.RosBridgeClient.TransferFromRosHandler handler = new RosSharp.RosBridgeClient.TransferFromRosHandler();

        string assetPath = Path.Combine(Path.GetFullPath("."), "Assets", "Urdf", "Models", robot.name);

        assetsRootDirectoryName = handler.ImportAssetsFromUrdf(protocol, this.RobotName, @"ws://localhost:9090",  20, assetPath, urdf);



        if (RobotRootObject == null)
        {
            RobotRootObject = new GameObject(RobotName);
        }

        RobotBuilder builder = new RobotBuilder();
        builder.Synchronize(robot, RobotRootObject, assetsRootDirectoryName);


        foreach (Rigidbody rb in RobotRootObject.GetComponentsInChildren<Rigidbody>())
            rb.isKinematic = true;

        if (SceneContainerGameObject != null)
        {
            RobotRootObject.transform.SetParent(SceneContainerGameObject.transform);
        }

        hasUrdfAssetsImported = true;
    }

    private void ReportSynch()
    {
        if (synchronizationCounterMax == -1) return;

        if (synchronizationCounterWithoutFullRegeneration == synchronizationCounterMax)
        {
            GameObject.Destroy(RobotRootObject);
            RobotRootObject = null;
            synchronizationCounterWithoutFullRegeneration = 0;
        }
    }



    internal void Destroy()
    {
        GameObject.Destroy(RobotRootObject);
    }
}
