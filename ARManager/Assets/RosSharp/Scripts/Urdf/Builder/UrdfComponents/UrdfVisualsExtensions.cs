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
    public static class UrdfVisualsExtensions
    {
        public static void Synchronize(Transform parent, List<Link.Visual> visuals = null)
        {
            if (parent.FindChildOrCreateWithComponent<UrdfVisuals>("Visuals", out GameObject visualsObject, out UrdfVisuals urdfVisuals))
            {
                visualsObject.hideFlags = HideFlags.NotEditable;
                urdfVisuals.hideFlags = HideFlags.None;
            }

            //Create all visuals that are in the list but do not yet exist
            if (visuals != null)
            {
                foreach (Link.Visual visual in visuals)
                {
                    UrdfVisualExtensions.Create(urdfVisuals.transform, visual);
                }
            }

            //Destroy existing visuals that are not in the list
            UrdfVisual[] vis = visualsObject.GetComponentsInChildren<UrdfVisual>();
            for (int i = vis.Length - 1; i >= 0; i--)
            {
                var visual = vis[i];
                if (!visuals.Any(x => x.name == visual.name))
                {
                    Object.Destroy(visual);
                }
            }
        }

        public static List<Link.Visual> ExportVisualsData(this UrdfVisuals urdfVisuals)
        {
            UrdfVisual[] urdfVisualsList = urdfVisuals.GetComponentsInChildren<UrdfVisual>();

            return urdfVisualsList.Select(urdfCollision => urdfCollision.ExportVisualData()).ToList();
        }
    }
}

