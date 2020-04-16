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

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfCollisionsExtensions
    {
        public static void Synchronize(Transform parent, List<Link.Collision> collisions = null)
        {
            if (parent.FindChildOrCreateWithComponent<UrdfCollisions>("Collisions", out GameObject collisionsObject, out UrdfCollisions urdfCollisions))
            {
                //created the object
                collisionsObject.hideFlags = HideFlags.NotEditable;
                urdfCollisions.hideFlags = HideFlags.None;
            }
            else
            {
                //object existed already
            }


            if (collisions != null)
            {
                foreach (Link.Collision collision in collisions)
                    UrdfCollisionExtensions.Create(urdfCollisions.transform, collision);
            }

            //Remove removed children
            var existingCollisions = collisionsObject.GetComponentsInSelf<UrdfCollision>();
            existingCollisions.RemoveAll(x => collisions.Any(y => y.name == x.name));
            Utils.DestroyAll(existingCollisions);
        }

        public static List<Link.Collision> ExportCollisionsData(this UrdfCollisions urdfCollisions)
        {
            UrdfCollision[] urdfCollisionsList = urdfCollisions.GetComponentsInChildren<UrdfCollision>();
            return urdfCollisionsList.Select(urdfCollision => urdfCollision.ExportCollisionData()).ToList();
        }
    }
}