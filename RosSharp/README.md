# [<img src="https://github.com/siemens/ros-sharp/wiki/img/Home_RosSharpLogo.png" width="480" alt ="ROS#"/>](https://github.com/siemens/ros-sharp) #

[ROS#](https://github.com/siemens/ros-sharp) is a set of open source software libraries and tools in [C\# ](https://docs.microsoft.com/de-de/dotnet/csharp/csharp) for communicating with [ROS](http://www.ros.org/) from .[NET](https://www.microsoft.com/net) applications, in particular [Unity](https://unity3d.com/).

Find some examples what you can do with ROS# [here](https://github.com/siemens/ros-sharp/wiki/Info_Showcases).

## Notes On this subfolder ##
This is a copy of the fork of ROS# from [here](https://github.com/EricVoll/ros-sharp)
## Change 1: URDF Parsing capabilities ##
This fork has some changes to allow a broader range of XML-Tags to be imported in the URDF files. The original repository included classes called "Plugins" that were used whenever no matching class was found. This fork allows to define XML-classNames that are mapped into instances of defined classes. Like the main ROS# branch, use 2019.x or later.

```
  //Attach tooltip factory to the Robot Parser
  var tooltipFactory = new AttachableComponentFactory<IAttachableComponent>("tooltip")
  {
      Constructor = () => new RosSharp.Urdf.Attachables.AttachedDataValue(),
  };

  Robot.attachableComponentFactories.Add(tooltipFactory);
```

This factory will now parse xml-classes such as:
```
<tooltip name="MyCoolTooltip">
  <parent name="SomeJoint"/>
  <origin .../>
</tooltip>
```
The XML-Parser will now create instances of `AttachedDataValue` class, and fill its values via reflection. The `Robot`-Object has the according fields 

Similarly to the other objects in the Robot class (Links, Joints, etc.), the `IAttachableComponent` contains these fields:
``` 
    public interface IAttachableComponent
    {
        string name { get; set; }
        string parent { get; set; }
        Origin origin { get; set; }
        Link parentLink { get; set; }
    }
```
Which can of course be extended by the class implementing the interface
```
    public class AttachedDataValue : IAttachableComponent
    {
        public AttachedDataValue();

        public string name { get; set; }
        public string parent { get; set; }
        public Origin origin { get; set; }
        public string topic { get; set; }
        public Link parentLink { get; set; }
    }
```
The AttachableComponentFactory offers methods to extract all objects that were found in the urdf file:
`public List<K> ExtractAttachableComponents<K>(Robot robot) where K : IAttachableComponent;`

## Change 2: Unity GameObject Creation from Robot objects ##
On the Unity side this fork contains two main features:
- The code was adjusted to run with multiple robot instances during runtime
- The code can now synchronize Robots with URDF files, instead of only creating them. This means, that if a URDF file dynamically changes (for example a sensor was attached during the runtime), the new urdf can be passed into the `RobotBuilder` and it adjusts updates the existing GameObject with it.

## Change 3: AOT compilability
In order to build for the HoloLens using the IL2CPP compiler, all libraries used must be AOT compilable. Even though many aspects of the System.Reflection namespace should not make problems, it did not work in our case. Thus, this repo has all System.Reflection parts removed and the issues were solved in another way.

## Change 4: Unity.Editor Mode
Alot of the original ROS# repo was written for the Editor mode, which means, that alot of the functionalieties would not work in built and deployed apps. This was fixed here.

## Original ReadMe:
#### Installation ### 

To use ROS# with the HoloLens, simply clone this fork and stay on the master branch. Then open the Unity project and import the [Microsoft Mixed Reality Toolkit](https://github.com/Microsoft/MixedRealityToolkit-Unity). The toolkit provides a version of Newtonsoft.Json that works on the HoloLens, as well as other tools/features you will want during HoloLens development. Follow the Mixed Reality Toolkit configuration instructions, and you will be good to go. I use the 2017 version of MRTK, not vNext.

### Architecture ###

How does this work under the hood? In brief, I wrote a UWP-compatible WebSocket interface for ROS#, created a UWP-compatible version of RosBridgeClient.dll, called RosBridgeClientUWP.dll, added that to the Unity Project, specified proper platforms for all .dlls, and edited RosConnector.cs to automatically use to the UWP WebSocket interface.

#### Creating RosBridgeClientUWP.dll ####

ROS# contains a solution in the Libraries folder, which contains a project called RosBridgeClient. In the Protocols folder, I created a UWP compatible WebSocket interface. Next, I wrapped all WebSocket protocol files in preprocessor directives so only the UWP compatible interface would be compiled in a UWP-build. Finally, I created a second project in the solution called RosBridgeClientUWP. I made it a Windows Universal class library project, and copied all of the RosBridgeClient code over as links. Copying as links means that editing the code in one location changes it in both. Finally, I built the solution.

#### Preparing the Unity Project ####

In the ROS# Unity project, I first installed the Mixed Reality Toolkit and followed their configuration instructions. Next, in RosSharp/Plugins, I deselected all platforms from Newtonsoft.Json, and excluded WSAPlayer from all others. Next, I copied over the RosBridgeClientUWP.dll into RosSharp/Plugins, and selected WSAPlayer as the only platform. Finally, I modified RosConnector.cs, using preprocessor directives to make Unity use the WebSocketUWP protocol if WINDOWS_UWP is defined.

### Making Changes ###

If you want to make changes to the RosBridgeClient, like adding new messages, for instance, simply edit the code in the RosBridgeClient project (following the instructions from the main ROS# wiki), build the solution, and copy over the new RosBridgeClient.dll and RosBridgeClientUWP.dll.


### Compatibile With Mixed Reality Toolkit ###
This branch is compatible with Microsoft's Mixed Reality Toolkit. See the Preparing Unity Project Section.

## Recent Changes ##

[This](https://github.com/siemens/ros-sharp/commit/acdd1ea7b8de47a23fbf376fa590590cf945b495) commit comes with major changes in how ROS# deals with URDF import/export

The biggest changes are:
* [Urdf Libary](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf): The UrdfImporter project was renamed to Urdf. It now supports the ability to both read from and write to URDF files.
* [Create, Modify, and Export URDF models in Unity](https://github.com/siemens/ros-sharp/tree/master/Unity3D): ROS# now supports creating and exporting URDF models directly in Unity. It is also possible to modify and re-export an existing URDF model.
* [Transfer URDF files from Unity to ROS](https://github.com/siemens/ros-sharp/wiki/User_App_ROS_TransferURDFToROS): Previously it was only possible to transfer/import URDF files from ROS to Unity. Now ROS# can send a URDF and all its meshes from Unity to a package in ROS.

Please see the [Wiki](https://github.com/siemens/ros-sharp/wiki/), especially [Section 3.2](https://github.com/siemens/ros-sharp/wiki/User_App_NoROS_ExportURDFOnWindows), for an explanation of how to use the new framework.

## Contents ##

* [Libraries](https://github.com/siemens/ros-sharp/tree/master/Libraries):
 .NET solution for [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient), [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
* [ROS](https://github.com/siemens/ros-sharp/tree/master/ROS):  [ROS](http://wiki.ros.org/) packages used by ROS#.
* [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D): [Unity](https://unity3d.com/) project containing
  * Unity-specific extensions to
   [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and
   [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/UrdfImporter)
  * example scenes and reference code (see [Wiki](https://github.com/siemens/ros-sharp/wiki))

## Releases ##

In addition to the source code, [Releases](https://github.com/siemens/ros-sharp/releases) contain:

* a [Unity Asset Package](https://docs.unity3d.com/Manual/AssetPackages.html) containing the [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) project assets:
  * to be imported in other Unity projects using ROS#.
* binaries of [RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) and [Urdf](https://github.com/siemens/ros-sharp/tree/master/Libraries/Urdf)
  * to be used in other .NET projects using these libraries.

The latest release is also being published in the [Unity Asset Store](https://assetstore.unity.com/packages/tools/physics/ros-ros-unity-communication-package-107085).

Please get the development version with latest changes and fixes directly from the [tip of this master branch](https://github.com/siemens/ros-sharp).

## Licensing ##

ROS# is open source under the [Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0) and is free for commercial use.

## External Dependencies ##

[RosBridgeClient](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient) requires:
* `websocket-sharp.dll` from [websocket-sharp](https://github.com/sta/websocket-sharp) provided under MIT License (required only when using [WebSocketSharpProtocol](https://github.com/siemens/ros-sharp/tree/master/Libraries/RosBridgeClient/Protocols/WebSocketSharpProtocol.cs)).
* `Newtonsoft.Json.dll` from [NewtonSoft Json.Net](http://www.newtonsoft.com/json) provided under MIT License.

## Platform Support ##

* [ROS#](https://github.com/siemens/ros-sharp) is developed for Windows and has successfully been used on Linux and iOS by community members.

* The [RosSharp](https://github.com/siemens/ros-sharp/tree/master/Libraries/) solution requires .NET Framework 4.6 and Visual Studio 2017 to compile.
* The Unity Project [Unity3D](https://github.com/siemens/ros-sharp/tree/master/Unity3D) requires Unity Version 2018.2 and higher.
Make sure to set the scripting runtime version to `.NET 4.x Equivalent` ([see Wiki page](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)).

* Please find a UWP version of ROS# [here](https://github.com/dwhit/ros-sharp).
* Please find a .NET Standard 2.0 version of UrdfImporter [here](https://github.com/blommers/UdrfImporter).

## Further Info ##

* [Read the Wiki](https://github.com/siemens/ros-sharp/wiki)
* [Contact the project team](mailto:ros-sharp.ct@siemens.com)
* [Contributors and Acknowledgements](https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements)

---

© Siemens AG, 2017-2018

Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
