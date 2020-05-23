using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClosePanel : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject Obj;
    public GameObject depthBacking;
    public void ClosePanelfunc()
    {
        if (Obj != null)
        {
            if (Obj.activeSelf == true)
            {
                Obj.SetActive(false);
            }
        }
        if (depthBacking != null)
        {
            if (depthBacking.activeSelf == true)
            {
                depthBacking.SetActive(false);
            }
        }
    }
}
 