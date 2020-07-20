using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Linq;
using static RosSharp.Urdf.Joint;

namespace RosSharp.Urdf
{
    public class AttachableComponent<T> where T : IAttachableComponent
    {
        /// <summary>
        /// The classname in the XML tree structure
        /// </summary>
        public string className;
        public T component;

        public AttachableComponent(string className, T instance)
        {
            this.component = instance;
            this.className = className;
        }


        /// <summary>
        /// Initializes all properties of <typeparamref name="T"/> via reflection
        /// </summary>
        /// <param name="node"></param>
        public void Initialize(XElement node)
        {
            component.topic = "/img/joint_position";

            component.name = (string)node.Attribute("name");
            component.parent = (string)node.Element("parent").Attribute("link");
            component.topic = (string)node.Attribute("topic");
        }
        
    }
}
