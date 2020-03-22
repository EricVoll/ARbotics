using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.UrdfTransfer;

public class URDFImporter : MonoBehaviour
{
    public bool DoImport = false;

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (DoImport)
        {

            string uri = "ws://192.168.119.129:9090";
            
            RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol webSocketNetProtocol = new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
            RosSocket rosSocket = new RosSocket(webSocketNetProtocol);
            string urdfParameter = "/robot_description";

            // Publication:
            UrdfTransferFromRos urdfTransferFromRos = new UrdfTransferFromRos(rosSocket, System.IO.Directory.GetCurrentDirectory(), urdfParameter);
            urdfTransferFromRos.Transfer();

            urdfTransferFromRos.Status["robotNameReceived"].WaitOne();
            Debug.Log("Robot Name Received: " + urdfTransferFromRos.RobotName);

            urdfTransferFromRos.Status["robotDescriptionReceived"].WaitOne();
            Debug.Log("Robot Description received... ");

            urdfTransferFromRos.Status["resourceFilesReceived"].WaitOne();
            Debug.Log("Resource Files received " + urdfTransferFromRos.FilesBeingProcessed.Count);

            rosSocket.Close();

            Debug.Log("Press any key to close...");

            DoImport = false;
        }
    }
}
