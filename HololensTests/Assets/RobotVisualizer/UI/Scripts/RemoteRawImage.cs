using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System.Threading.Tasks;
using UnityEngine.UI;

public class RemoteRawImage : MonoBehaviour
{

    [SerializeField] string _imageUrl;
    [SerializeField] private GameObject anchor;

    // Next update in second
    private double nextUpdate = 1;
    private double updateRate = 0.2;
    private RawImage m_RawImage;


    // Start is called before the first frame update
    void Start()
    {
        m_RawImage = GetComponent<RawImage>();
    }

    // Update is called once per frame
    void Update()
    {
        // If the next update is reached
        if (Time.time >= nextUpdate) {
            nextUpdate = Time.time + updateRate;
            TimerFunction();
        }
    }

    // Update is called once per second
    async void TimerFunction() {
        Texture2D tex = await GetRemoteTexture(_imageUrl);
        // RenderTexture rt = new RenderTexture(tex.width, tex.height, 24, RenderTextureFormat.ARGB32);
        // rt.filterMode = FilterMode.Trilinear;
        m_RawImage.texture = tex;
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
