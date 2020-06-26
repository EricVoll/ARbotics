using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MMOpen : MonoBehaviour
{
    public void OpenPanelfuncMM()
    {
        if (MainMenuOpener.Obj != null)
        {
            if (MainMenuOpener.Obj.activeSelf == false)
            {
                MainMenuOpener.Obj.SetActive(true);
            }
        }
    }
}
