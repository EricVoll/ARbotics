using RosSharp.Urdf;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UrdfSyncher : MonoBehaviour
{
    /// <summary>
    /// 
    /// </summary>
    /// <param name="sceneContainerObject">The gameobject which should be used to contain this robot</param>
    public UrdfSyncher(GameObject sceneContainerObject)
    {
        SceneContainerGameObject = sceneContainerObject; 
    }

    /// <summary>
    /// The Root gameobject which will be used to spawn all robots. This will be the parent
    /// </summary>
    public GameObject SceneContainerGameObject;

    /// <summary>
    /// The root gameobject of this robot
    /// </summary>
    private GameObject RobotRootObject;
    
    private bool hasInitialUrdfModelImported= false;


    /// <summary>
    /// Compares the current devices in the scene with the urdf file and performs adjustments
    /// </summary>
    /// <param name="urdf"></param>
    public void SynchUrdf(string urdf)
    {
        if(hasInitialUrdfModelImported == false)
        {
            ImportInitialUrdfModel();
        }


        var tooltipFactory = new AttachableComponentFactory<IAttachableComponent>("tooltip")
        {
            Constructor = () => new RosSharp.Urdf.Attachables.AttachedDataValue(),
        };

        Robot.attachableComponentFactories.Add(tooltipFactory);

        Robot robot = Robot.FromContent(urdf);

        if (RobotRootObject == null) RobotRootObject = new GameObject($"RobotRoot_{robot.name}");

        RobotBuilder builder = new RobotBuilder();
        builder.Synchronize(robot, RobotRootObject);
    }

    private void ImportInitialUrdfModel()
    {

    }
}
