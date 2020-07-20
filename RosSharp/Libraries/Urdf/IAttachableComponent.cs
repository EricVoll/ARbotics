using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RosSharp.Urdf
{
    public interface IAttachableComponent
    {
        string name { get; set; }
        string parent { get; set; }
        string topic { get; set; }
        Origin origin { get; set; }

        Link parentLink { get; set; }
    }
}
