using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.Urdf
{
    public class AttachableComponentFactory<T> where T : IAttachableComponent
    {
        public AttachableComponentFactory(string className)
        {
            this.ClassName = className;
        }

        public string ClassName { get; set; }

        public Func<T> Constructor { get; set; }

        public AttachableComponent<T> ConstructAttachableInstance()
        {
            AttachableComponent<T> obj = new AttachableComponent<T>(ClassName, Constructor());
            return obj;
        }

        public List<K> ExtractAttachableComponents<K>(Robot robot) where K : IAttachableComponent
        {
            List<K> list;

            list = robot.attachedComponents.Where(x => x.className == this.ClassName)
                .Select(x => x.component)
                .Cast<K>()
                .ToList();

            return list;
        }
    }
}
