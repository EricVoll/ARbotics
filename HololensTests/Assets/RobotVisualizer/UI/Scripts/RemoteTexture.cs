using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Networking;

public class RemoteTexture : MonoBehaviour {

    [SerializeField] string _imageUrl;
    [SerializeField] Material _material;

    // Start is called before the first frame update
    async void Start() {
        Texture2D texture = await GetRemoteTexture(_imageUrl);
        _material.mainTexture = texture;
    }

    // Update is called once per frame
    void Update() {

    }


    public static async Task<Texture2D> GetRemoteTexture(string url) {
        using (UnityWebRequest www = UnityWebRequestTexture.GetTexture(url)) {
            //begin requenst:
            var asyncOp = www.SendWebRequest();

            //await until it's done: 
            while (asyncOp.isDone == false) {
                await Task.Delay(1000 / 30);//30 hertz
            }

            //read results:
            if (www.isNetworkError || www.isHttpError) {

                //log error:
                #if DEBUG
                Debug.Log($"{ www.error }, URL:{ www.url }");
                #endif

                //nothing to return on error:
                return null;
            } else {
                //return valid results:
                return DownloadHandlerTexture.GetContent(www);
            }
        }
    }

}
