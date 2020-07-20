using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.UrdfTransfer;
using RosSharp.Urdf;
using RosSharp.Urdf.Attachables;
using System;
using Xunit;

namespace UrdfTests
{
    public class AttachableComponentTests
    {
        [Fact]
        public void ShouldCreateInstance()
        {
            string xml = UrdfTests.Properties.Resources.anymal;

            AttachableComponentFactory<IAttachableComponent> factory =
                new AttachableComponentFactory<IAttachableComponent>("tooltip")
                {
                    Constructor = () => new AttachedDataValue(),
                };

            Robot.attachableComponentFactories.Add(factory);

            Robot robot = Robot.FromContent(xml);

            var dataValues = factory.ExtractAttachableComponents<AttachedDataValue>(robot);

            Assert.True(robot.attachedComponents.Count > 0);
            Assert.True(robot.attachedComponents[0] != null);
            Assert.True((robot.attachedComponents[0].component as AttachedDataValue).topic == "/test/topic");
            //Assert.True(robot.attachedComponents[0].parentLink != null);
            //Assert.True(robot.attachedComponents[0].component != null);
        }

        [Fact]
        public void ShouldCreateFullRobot()
        {
            string xml = UrdfTests.Properties.Resources.xmlResFullRobot;
            Robot r = Robot.FromContent(xml);

        }

        [Fact]
        public void ShouldCreateInstance_WithToolTip()
        {
            string xml = UrdfTests.Properties.Resources.xmlResSingleNode;

            AttachableComponentFactory<IAttachableComponent> factory =
                new AttachableComponentFactory<IAttachableComponent>("tooltip")
                {
                    Constructor = () => new AttachedDataValue(),
                };

            Robot.attachableComponentFactories.Add(factory);

            Robot robot = Robot.FromContent(xml);

            var dataValues = factory.ExtractAttachableComponents<AttachedDataValue>(robot);

            Assert.True(robot.attachedComponents.Count > 0);
            Assert.True(robot.attachedComponents[0] != null);
            Assert.True((robot.attachedComponents[0].component as AttachedDataValue).topic == "/test/topic");
            //Assert.True(robot.attachedComponents[0].parentLink != null);
            //Assert.True(robot.attachedComponents[0].component != null);
        }

        [Fact]
        public void ShouldImportFilesDirectly()
        {
            string uri = "ws://localhost:9090";
            RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol webSocketNetProtocol = new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(uri);
            RosSocket rosSocket = new RosSocket(webSocketNetProtocol);
            string urdfParameter = "/robot_description";
            string t = JsonConvert.SerializeObject(rosSocket);

            // Publication:
            UrdfTransferFromRos urdfTransferFromRos = new UrdfTransferFromRos(rosSocket, System.IO.Directory.GetCurrentDirectory(), urdfParameter);
            //urdfTransferFromRos.Transfer();
            urdfTransferFromRos.RobotName = "UR3";
            urdfTransferFromRos.ImportResourceFiles(UrdfTests.Properties.Resources.xmlUr5);
            urdfTransferFromRos.Status["resourceFilesReceived"].WaitOne();
            Console.WriteLine("Resource Files received " + urdfTransferFromRos.FilesBeingProcessed.Count);

            rosSocket.Close();
        }
    }
}
