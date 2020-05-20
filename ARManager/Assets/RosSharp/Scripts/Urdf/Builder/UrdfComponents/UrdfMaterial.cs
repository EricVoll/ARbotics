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
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System.Linq;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfMaterial
    {
        private const string DefaultMaterialName = "Default";
        private const int RoundDigits = 4;

        public static Dictionary<string, Link.Visual.Material> Materials =
            new Dictionary<string, Link.Visual.Material>();

        /// <summary>
        /// A dictionery containing all created materials. The key : string is the name of the urdf material
        /// </summary>
        public static Dictionary<string, Material> MaterialsCreated = new Dictionary<string, Material>();


        #region Import
        private static Material CreateMaterial(this Link.Visual.Material urdfMaterial)
        {
            if (MaterialsCreated.ContainsKey(urdfMaterial.name))
            {
                return MaterialsCreated[urdfMaterial.name];
            }

            if (urdfMaterial.name == "")
                urdfMaterial.name = GenerateMaterialName(urdfMaterial);

            //var material = AssetDatabase.LoadAssetAtPath<Material>(UrdfAssetPathHandler.GetMaterialAssetPath(urdfMaterial.name));
            var all = Resources.LoadAll("", typeof(Material));
            var material = all.FirstOrDefault(x => x.name == urdfMaterial.name);

            if (material != null) //material already exists
            {
                MaterialsCreated[urdfMaterial.name] = material as Material;
                return material as Material;
            }
            else
            {
                return new Material(Shader.Find("Specular"));
            }
        }
        

        private static Material InitializeMaterial()
        {
            var material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            return material;
        }

        private static string GenerateMaterialName(Link.Visual.Material urdfMaterial)
        {
            var materialName = "";
            if (urdfMaterial.color != null)
            {
                materialName = "rgba-";
                for (var i = 0; i < urdfMaterial.color.rgba.Length; i++)
                {
                    materialName += urdfMaterial.color.rgba[i];
                    if (i != urdfMaterial.color.rgba.Length - 1)
                        materialName += "-";
                }
            }
            else if (urdfMaterial.texture != null)
                materialName = "texture-" + Path.GetFileName(urdfMaterial.texture.filename);

            return materialName;
        }

        private static Color CreateColor(Link.Visual.Material.Color urdfColor)
        {
            return new Color(
                (float)urdfColor.rgba[0],
                (float)urdfColor.rgba[1],
                (float)urdfColor.rgba[2],
                (float)urdfColor.rgba[3]);
        }

        private static Texture LoadTexture(string filename)
        {
            return filename == "" ? null : LocateAssetHandler.FindUrdfAsset<Texture>(filename);
        }

        public static void SetUrdfMaterial(GameObject gameObject, Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial != null)
            {
                var material = CreateMaterial(urdfMaterial);
                SetMaterial(gameObject, material);
            }
            else
            {
                //If the URDF material is not defined, and the renderer is missing
                //a material, assign the default material.
                Renderer renderer = gameObject.GetComponentInChildren<Renderer>();
                if (renderer != null && renderer.sharedMaterial == null)
                {
                    var defaultMaterial = new Material(Shader.Find("Specular"));
                    SetMaterial(gameObject, defaultMaterial);
                }
            }
        }

        private static void SetMaterial(GameObject gameObject, Material material)
        {
            var renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (var renderer in renderers)
                renderer.sharedMaterial = material;
        }
        #endregion
        
    }
}