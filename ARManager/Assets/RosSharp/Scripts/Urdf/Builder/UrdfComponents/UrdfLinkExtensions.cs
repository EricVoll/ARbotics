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
using System.Linq;
using UnityEngine;
using RosSharp;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfLinkExtensions
    {
        public static UrdfLink Synchronize(Transform parent, Link link = null, Joint joint = null)
        {
            parent.FindChildOrCreateWithComponent<UrdfLink>(link != null ? link.name : Utils.GenerateNonReferenceID(link), out GameObject linkObject, out UrdfLink urdfLink);

            if (link != null)
            {
                foreach (var attachedComponent in link.attachableComponents)
                {
                    AttachedDataSynchronizer.Instance.HandleAttachedComponent(linkObject, attachedComponent, link);
                }
            }

            UrdfVisualsExtensions.Synchronize(linkObject.transform, link?.visuals);
            UrdfCollisionsExtensions.Synchronize(linkObject.transform, link?.collisions);

            if (link != null)
                urdfLink.ImportLinkData(link, joint);
            else
            {
                UrdfInertial.Synchronize(linkObject);
                UnityEditor.EditorGUIUtility.PingObject(linkObject);
            }

            //Remove Attached Values that are too much
            var attachedValueChildren = linkObject.GetComponentsInDirectChildrenFromGameobject<AttachedValue>();
            attachedValueChildren.RemoveAll(x => link.attachableComponents.Any(y => y.component.name == x.name));
            Utils.DestroyAll(attachedValueChildren.Select(x => x.gameObject));

            return urdfLink;
        }

        private static void ImportLinkData(this UrdfLink urdfLink, Link link, Joint joint)
        {
            if (link.inertial == null && joint == null)
                urdfLink.IsBaseLink = true;

            urdfLink.gameObject.name = link.name;

            if (joint?.origin != null)
                UrdfOrigin.ImportOriginData(urdfLink.transform, joint.origin);

            if (link.inertial != null)
            {
                UrdfInertial.Synchronize(urdfLink.gameObject, link.inertial);

                if (joint != null)
                    UrdfJoint.Synchronize(urdfLink.gameObject, UrdfJoint.GetJointType(joint.type), joint);
            }
            else if (joint != null)
                Debug.LogWarning("No Joint Component will be created in GameObject \"" + urdfLink.gameObject.name + "\" as it has no Rigidbody Component.\n"
                                 + "Please define an Inertial for Link \"" + link.name + "\" in the URDF file to create a Rigidbody Component.\n", urdfLink.gameObject);

            foreach (Joint childJoint in link.joints.Where(x => x.ChildLink != null))
            {
                Link child = childJoint.ChildLink;
                UrdfLinkExtensions.Synchronize(urdfLink.transform, child, childJoint);
            }

            var linkChildren = Utils.GetComponentsInDirectChildrenFromGameobject<UrdfLink>(urdfLink.gameObject);
            List<string> wantedObjectNames = link.joints
                .Where(x => x.ChildLink != null)
                .Select(x => String.IsNullOrEmpty(x.child) ? Utils.GenerateNonReferenceID(x.ChildLink) : x.child)
                .ToList();

            linkChildren.RemoveAll(x => wantedObjectNames.Contains(x.name));
            Utils.DestroyAll(linkChildren.Select(x => x.gameObject));
        }

        public static Link ExportLinkData(this UrdfLink urdfLink)
        {
            if (urdfLink.transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + urdfLink.gameObject.name + "\" cannot be saved to the URDF file.", urdfLink.gameObject);

            UrdfInertial urdfInertial = urdfLink.gameObject.GetComponent<UrdfInertial>();
            Link link = new Link(urdfLink.gameObject.name)
            {
                visuals = urdfLink.GetComponentInChildren<UrdfVisuals>().ExportVisualsData(),
                collisions = urdfLink.GetComponentInChildren<UrdfCollisions>().ExportCollisionsData(),
                inertial = urdfInertial == null ? null : urdfInertial.ExportInertialData()
            };

            return link;
        }
    }
}