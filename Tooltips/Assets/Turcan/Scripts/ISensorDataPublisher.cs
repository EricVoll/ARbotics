using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface ISensorDataPublisher 
{
    Action<Vector3[]> ExtractPointCallBack { get; set; }
}
