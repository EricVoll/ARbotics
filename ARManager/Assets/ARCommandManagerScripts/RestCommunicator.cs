using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RestCommunicator : MonoBehaviour
{
    public TextAsset GetAvailableComponentsTestResponse;


    const string RestServerUrl = "http://127.17.0.1:5000/";
    public enum RequestUrl
    {
        AvailableComponents,
        Instances
    };
    public enum RequestType
    {
        Get,
        Post
    };
    Dictionary<RequestUrl, string> RestUrl = new Dictionary<RequestUrl, string>()
    {
        { RequestUrl.AvailableComponents,  RestServerUrl + "AvailComps" },
        { RequestUrl.Instances, RestServerUrl + "Instances" },
    };

    /// <summary>
    /// Retrieves all available robots and returns an object with the information returned from the server
    /// </summary>
    /// <returns></returns>
    public ARCommandAvailableCompoenntResponse RequestAvailableRobots()
    {
        //Send post request
        string responseJson = SendRestRequest(RequestUrl.AvailableComponents, RequestType.Get);

        ARCommandAvailableCompoenntResponse response = JsonConvert.DeserializeObject<ARCommandAvailableCompoenntResponse>(responseJson);

        return response;
    }

    /// <summary>
    /// Requests a new instance of the specified component
    /// </summary>
    /// <param name="instanceName"></param>
    public void RequestNewComponentIntsance(string componentName)
    {
        object requestObject = new
        {
            comp_name = componentName
        };

        SendRestRequest(RequestUrl.Instances, RequestType.Post, requestObject);
    }

    /// <summary>
    /// Sends 
    /// </summary>
    /// <param name="url"></param>
    /// <param name="requestObject"></param>
    /// <returns></returns>
    public string SendRestRequest(RequestUrl url, RequestType type, object requestObject = null)
    {
        string requestUrl = RestUrl[url];


        switch (type)
        {
            case RequestType.Get:
                //Send get request
                break;
            case RequestType.Post:
                if (requestObject != null)
                {
                    string json = JsonConvert.SerializeObject(requestObject);
                    //Send request with json body
                }
                else
                {
                    //Send request without json body
                }
                break;
            default:
                break;
        }


        if (url == RequestUrl.AvailableComponents && type == RequestType.Get)
        {
            return GetAvailableComponentsTestResponse.text;
        }

        return "";
    }


}


#region Get Available Components response

public class ARCommandAvailableCompoenntResponse
{
    public List<AvailableComponent> components;
}

public class AvailableComponent
{
    public string pretty_name { get; set; }
    public int max_instances { get; set; }
    public int instances { get; set; }
    public string comp_type { get; set; }
    public string urdf_stat { get; set; }
    public DockerStartupInformation docker { get; set; }
}

public class DockerStartupInformation
{
    public string cmd { get; set; }
    public string image { get; set; }
    public string network { get; set; }
    //public object ports { get; set; }
    //field disabled because "9090/tcp" is not a valid C# field name and can not be deserialized without using tricks
    //since we don't really need it, ignore it.
}


#endregion

#region Get Intsance Response classes



public class ARCommanderMessage
{
    //All available devices that are requestable by the Unity side.
    public List<AvailableRobot> availableDevices;

    //All devices that should be displayed
    public List<DeviceUrdf> devices;
}

public class AvailableRobot
{
    //FriendlyName for UI
    public string name;
    //Id to request the publisher to be started
    public string id;
    //URL where the Stationary Side will publish the urdf contents
    public string publishingServerUrl;
}

public class DeviceUrdf
{
    //URDF content of the device
    public string data;
    //The ID of the current instance
    public string id;
}

#endregion
