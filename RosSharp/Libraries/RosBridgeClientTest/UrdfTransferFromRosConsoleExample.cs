﻿/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

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

using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.UrdfTransfer;

// commands on ROS system:
// launch before starting:
// roslaunch file_server publish_description_turtlebot2.launch

namespace RosSharp.RosBridgeClientTest
{
    public class UrdfTransferFromRosConsoleExample
    {
        public static void Main(string[] args)
        {
            string uri = "ws://192.168.50.85:9090";
            //uri = "ws://127.0.0.1:9090";

            for (int i = 1; i < 3; i++)
            {
                try
                {

                    RosBridgeClient.Protocols.WebSocketNetProtocol webSocketNetProtocol = new RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
                    RosSocket rosSocket = new RosSocket(webSocketNetProtocol);
                    string urdfParameter = "/robot_description";

                    // Publication:
                    UrdfTransferFromRos urdfTransferFromRos = new UrdfTransferFromRos(rosSocket, System.IO.Directory.GetCurrentDirectory(), urdfParameter);
                    urdfTransferFromRos.Transfer();

                    urdfTransferFromRos.Status["robotNameReceived"].WaitOne();
                    Console.WriteLine("Robot Name Received: " + urdfTransferFromRos.RobotName);

                    urdfTransferFromRos.Status["robotDescriptionReceived"].WaitOne();
                    Console.WriteLine("Robot Description received... ");

                    urdfTransferFromRos.Status["resourceFilesReceived"].WaitOne();
                    Console.WriteLine("Resource Files received " + urdfTransferFromRos.FilesBeingProcessed.Count);

                    rosSocket.Close();
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                    Console.WriteLine("Waiting for input to start next attempt");
                    Console.ReadLine();
                }
            }

            Console.WriteLine("Press any key to close...");
            Console.ReadKey(true);
        }

    }
}