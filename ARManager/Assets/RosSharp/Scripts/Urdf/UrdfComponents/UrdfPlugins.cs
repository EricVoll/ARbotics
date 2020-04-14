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

namespace RosSharp.Urdf
{
    public class UrdfPlugins : MonoBehaviour
    {
        public static void Create(Transform robot, List<Plugin> plugins = null)
        {
            if(robot.FindChildOrCreate("Plugins", out GameObject pluginsObject))
            {
                pluginsObject.AddComponent<UrdfPlugins>();
            }

            //Add or check all plugins
            foreach (var plugin in plugins)
                UrdfPlugin.Create(pluginsObject.transform, plugin);

            //Remove all plugins that are not in the plugin list
            var existingPlugins = pluginsObject.GetComponentsInChildren<UrdfPlugin>();

            if (existingPlugins != null)
            {
                for (int i = existingPlugins.Length - 1; i >= 0; i--)
                {
                    if (!plugins.Any(x => x.text == existingPlugins[i].PluginText))
                    {
                        //None of the new plugins contain this plugintext
                        Destroy(existingPlugins[i]);
                    }
                }
            }
        }


        public List<Plugin> ExportPluginsData()
        {
            return GetComponents<UrdfPlugin>()
                .Select(urdfPlugin => urdfPlugin.ExportPluginData())
                .Where(plugin => plugin != null)
                .ToList();
        }
    }

}