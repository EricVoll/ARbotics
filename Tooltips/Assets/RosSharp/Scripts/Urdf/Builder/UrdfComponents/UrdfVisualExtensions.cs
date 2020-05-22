/*
© Siemens AG, 2018
Author: Suzannah Smith (suzannah.smith@siemens.com)

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
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfVisualExtensions
    {
        public static void Create(Transform parent, GeometryTypes type)
        {
            GameObject visualObject = new GameObject("unnamed");
            visualObject.transform.SetParentAndAlign(parent);
            UrdfVisual urdfVisual = visualObject.AddComponent<UrdfVisual>();

            urdfVisual.GeometryType = type;
            UrdfGeometryVisual.Create(visualObject.transform, type);
        }

        public static void Create(Transform parent, Link.Visual visual)
        {
            if (String.IsNullOrEmpty(visual.name))
                visual.name = visual.GenerateNonReferenceID();

            if (parent.FindChildOrCreateWithComponent<UrdfVisual>(visual.name, out GameObject visualObject, out UrdfVisual urdfVisual))
            {
                //only create these visuals if the gameobject had to be created itself
                urdfVisual.GeometryType = UrdfGeometry.GetGeometryType(visual.geometry);
                UrdfGeometryVisual.Create(visualObject.transform, urdfVisual.GeometryType, visual.geometry);
            }

            //update these values every time
            UrdfMaterial.SetUrdfMaterial(visualObject, visual.material);
            UrdfOrigin.ImportOriginData(visualObject.transform, visual.origin);
        }

        public static void AddCorrespondingCollision(this UrdfVisual urdfVisual)
        {
            UrdfCollisions collisions = urdfVisual.GetComponentInParent<UrdfLink>().GetComponentInChildren<UrdfCollisions>();
            UrdfCollisionExtensions.Create(collisions.transform, urdfVisual.GeometryType, urdfVisual.transform);
        }
        
    }
}