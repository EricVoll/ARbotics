using Newtonsoft.Json;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

namespace ARRobotInteraction.Base
{

    /// <summary>
    /// The RestCommunicator handles all the REST request to the REST API and provides some untiliy functions
    /// </summary>
    public class RestCommunicator : MonoBehaviour
    {
        /// <summary>
        /// The URL that will be used to send the requests to
        /// </summary>
        public string RestServerUrl = "http://127.17.0.1:5000/";

        public enum RequestUrl
        {
            AvailableComponents,
            Instances
        };

        Dictionary<RequestUrl, string> RestUrl;

        public void Awake()
        {
            RestUrl = new Dictionary<RequestUrl, string>() {
            { RequestUrl.AvailableComponents,  RestServerUrl + "AvailComps" },
            { RequestUrl.Instances, RestServerUrl + "Instances" },
        };
        }

        /// <summary>
        /// Retrieves all available robots and returns an object with the information returned from the server
        /// </summary>
        /// <returns></returns>
        public void RequestAvailableRobots(Action<ARCommandAvailableComponentResponse> responseProcessor)
        {
            //Send post request
            SendGetRequest<ARCommandAvailableComponentResponse>(RequestUrl.AvailableComponents, responseProcessor);
        }

        /// <summary>
        /// Requests a new instance of the specified component
        /// </summary>
        /// <param name="instanceName"></param>
        public void RequestNewComponentIntsance(string componentName, Action<bool> callBack)
        {
            object requestObject = new
            {
                comp_name = componentName
            };

            SendPostRequest(RequestUrl.Instances, callBack, requestObject);
        }

        /// <summary>
        /// Sends a Get-Request to the specified url without a body and calls the callback after completion
        /// </summary>
        /// <param name="url"></param>
        /// <param name="callBack">The callback called after completion with the return json deserialized into <typeparamref name="T"/></param>
        /// <returns></returns>
        public void SendGetRequest<T>(RequestUrl url, Action<T> callBack)
        {
            string requestUrl = RestUrl[url];

            StartCoroutine(Get(requestUrl, callBack));
        }


        /// <summary>
        /// Sends a Post-Request to the specified url and calls the optional callback after completion.
        /// </summary>
        /// <param name="callBack">An optional callback called after completion with the parameter being a boolean that indicates whether or not the request succeeded</param>
        /// <param name="url">The url to send the request to</param>
        /// <param name="requestObject">An optional object that is serialized and sent as a json body with the request</param>
        /// <returns></returns>
        public void SendPostRequest(RequestUrl url, Action<bool> callBack = null, object requestObject = null)
        {
            string requestUrl = RestUrl[url];

            if (requestObject != null)
            {
                string json = JsonConvert.SerializeObject(requestObject);
                //Send request with json body
                StartCoroutine(Post(requestUrl, json, callBack));
            }
            else
            {
                //Send request without json body
                StartCoroutine(Post(requestUrl, "", callBack));
            }
        }

        private IEnumerator Get<T>(string url, Action<T> callBack)
        {
            using (UnityWebRequest www = UnityWebRequest.Get(url))
            {
                yield return www.SendWebRequest();

                if (www.isNetworkError)
                {
                    Debug.LogError(www.error);
                }
                else
                {
                    if (www.isDone)
                    {
                        string jsonResult = System.Text.Encoding.UTF8.GetString(www.downloadHandler.data);

                        T res = JsonConvert.DeserializeObject<T>(jsonResult);

                        callBack(res);
                        Debug.Log(jsonResult);
                    }
                }
            }
        }

        private IEnumerator Post(string url, string payload, Action<bool> callBack = null)
        {

            var jsonBinary = System.Text.Encoding.UTF8.GetBytes(payload);

            DownloadHandlerBuffer downloadHandlerBuffer = new DownloadHandlerBuffer();

            UploadHandlerRaw uploadHandlerRaw = new UploadHandlerRaw(jsonBinary);
            uploadHandlerRaw.contentType = "application/json";

            UnityWebRequest www =
                new UnityWebRequest(url, "POST", downloadHandlerBuffer, uploadHandlerRaw);

            yield return www.SendWebRequest();

            if (www.isNetworkError)
                Debug.LogError(string.Format("{0}: {1}", www.url, www.error));
            else
                Debug.Log(string.Format("Response: {0}", www.downloadHandler.text));

            callBack?.Invoke(!www.isNetworkError);
        }

    }

    #region Get Available Components response

    public class ARCommandAvailableComponentResponse
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



    public class Instance
    {
        public int inst_id { get; set; }
        public double start_time { get; set; }
        public object stop_time { get; set; }
        public bool active { get; set; }
        public bool running { get; set; }
        public bool killed { get; set; }
        public bool stop { get; set; }
        public string urdf_dyn { get; set; }
    }

    public class Ports
    {
        //this wont work
        public int port { get; set; }
    }

    public class DockerMeshVolume
    {
        public string bind { get; set; }
        public string mode { get; set; }
    }

    public class Volumes
    {
        public DockerMeshVolume docker_MeshVolume { get; set; }
    }

    public class Docker
    {
        public string command { get; set; }
        public string image { get; set; }
        public string network { get; set; }
        public Ports ports { get; set; }
        public object environment { get; set; }
        public bool detach { get; set; }
        public Volumes volumes { get; set; }
    }

    public class Component
    {
        public string pretty_name { get; set; }
        public int max_instances { get; set; }
        public int instances { get; set; }
        public string comp_type { get; set; }
        public string urdf_stat { get; set; }
        public Docker docker { get; set; }
    }

    public class RunningInstance
    {
        public Instance inst { get; set; }
        public Component comp { get; set; }
    }

    #endregion



}