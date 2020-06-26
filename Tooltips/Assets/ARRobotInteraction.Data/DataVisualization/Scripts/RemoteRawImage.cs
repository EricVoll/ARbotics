using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using System.Threading.Tasks;
using UnityEngine.UI;

namespace ARRobotInteraction.Data
{

    public class RemoteRawImage : MonoBehaviour {

        public string _imageUrl;

        [Tooltip("If Anchor exists, the Image is applied as material. " +
        "Otherwise the script searches directly for a raw image component.")]
        public GameObject anchor;
        public Material templateMaterial;

        // Next update in second
        private double nextUpdate = 1;
        private double updateRate = 0.1;

        private RawImage rawImage;
        private Material material;

        private Texture2D previousTexture;
        private MeshRenderer renderer;


        // Start is called before the first frame update
        void Start() {
            if (anchor == null) {
                rawImage = GetComponent<RawImage>();
            } else {
                // this.material = new Material(Shader.Find("Mixed Reality Toolkit/Standard"));
                this.material = new Material(templateMaterial);
                this.renderer = this.anchor.GetComponent<MeshRenderer>();
                Material[] mats = this.renderer.materials;
                this.renderer.materials = new Material[] { this.material };
            }
        }

        // Update is called once per frame
        public async void Update() {
            this.material.SetColor("_Color", Color.white);
            // If the next update is reached
            if (Time.time >= nextUpdate) {
                nextUpdate = Time.time + updateRate;
                await TimerFunction();
            }
        }

        // Update is called once per second
        private async Task TimerFunction() {

            Texture2D tex = await GetRemoteTexture(_imageUrl);
            tex.filterMode = FilterMode.Bilinear;
            tex.hideFlags = HideFlags.HideAndDontSave;

            if (anchor == null) {
                if (rawImage != null) rawImage.texture = tex;
            } else {
                // Why not RenderTexture?
                // RenderTexture rt = new RenderTexture(tex.width, tex.height, 24, RenderTextureFormat.ARGB32);
                // rt.filterMode = FilterMode.Bilinear;

                // Trials
                this.renderer.material.mainTexture = tex;
            }

            Object.Destroy(this.previousTexture);
            this.previousTexture = tex;
        }

        private static async Task<Texture2D> GetRemoteTexture(string url) {
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
}
