using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OpenPanel : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject Obj;
    public void OpenPanelfunc()
    {
        if (Obj != null)
        {
            if (Obj.activeSelf == false)
            {
                Obj.SetActive(true);
            }
        }
    }
}
