using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace ARRobotInteraction.Data
{
    public class BoxColliderResizer : MonoBehaviour {

        public GameObject target;

        private BoxCollider collider;

        // Start is called before the first frame update
        void Start() {
            if (collider == null) {
                collider = gameObject.GetComponent(typeof(BoxCollider)) as BoxCollider;
                if (collider == null) {
                    collider = gameObject.AddComponent<BoxCollider>();
                }
            }
        }

        // Update is called once per frame
        void Update() {

            Bounds bounds = new Bounds(Vector3.zero, Vector3.zero);
            bool boundsInitialized = false;

            // MeshFilter filter = target.GetComponent<MeshFilter>();
            Renderer renderer = target.GetComponent<Renderer>();
            if (renderer != null) {
                // bounds.Encapsulate(renderer.bounds);
                bounds = new Bounds(renderer.bounds.center, renderer.bounds.size);
                boundsInitialized = true;
            }

            Transform[] allDescendants = gameObject.GetComponentsInChildren<Transform>();
            foreach (Transform desc in allDescendants) {
                MeshFilter cfilter = desc.GetComponent<MeshFilter>();
                Renderer crenderer = desc.GetComponent<Renderer>();

                if (crenderer != null && boundsInitialized == true) {
                    bounds.Encapsulate(crenderer.bounds);
                } else if (crenderer != null) {
                    bounds = new Bounds(crenderer.bounds.center, crenderer.bounds.size);
                    boundsInitialized = true;
                }
                collider.center = bounds.center;
                collider.size = bounds.size;
            }
        }
    }
}