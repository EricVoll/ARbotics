using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.Urdf.Attachables
{
    public class AttachedDataValue : IAttachableComponent
    {
        public string name { get; set; }
        public string parent { get; set; }
        public Origin origin { get; set; }
        public string topic { get; set; }
        public Link parentLink { get; set; }
    }
}
