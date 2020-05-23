using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainMenuOpener : MonoBehaviour
{
    // Start is called before the first frame update
    public static GameObject Obj;
    private void Start()
    {
        OpenPanelfunc();
    }
    public static void OpenPanelfunc()
    {
        GameObject[] MainMenu;
        MainMenu = GameObject.FindGameObjectsWithTag("MMenu");
        Obj = MainMenu[0];
        //if (Obj.activeSelf == false)
        //    {
        //    Obj.SetActive(true);
        //    }
        
    }
}

