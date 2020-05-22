using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class OpenPanelWithTimer : MonoBehaviour //, IPointerEnterHandler, IPointerExitHandler
{
    // Start is called before the first frame update
    public GameObject Obj;
    public GameObject depthBacking;
    public static float TimeToWait=2.5f;
    public static bool IsHovering = false;
    //float countdownLength = 1.0f;
    //private void Update()
    //{
    //    timeLeft -= Time.deltaTime;
    //    //Debug.Log(timeLeft);

    //    if (timeLeft < 0)
    //    {
    //        OpenPanelfuncWithTimer();
    //    }
    //}
    //public void OnPointerEnter(PointerEventData eventData)
    //{
    //    Debug.Log("MYYYYYYYYCalled");
    //}
    public void OpenPanelfuncWithTimer()
    {
        if (Obj != null)
        {
            //Debug.Log("Called");
            StartGameTimer();
            IsHovering = true;

        }
    }

    //public void OnPointerExit(PointerEventData eventData)
    //{
    //    throw new System.NotImplementedException();
    //}
    public void StartGameTimer()
    {
        //Debug.Log("StartGameTimer");
        /*        GameObject gameController = GameObject.FindGameObjectWithTag("gc");   */              // First, find the GameObject
        //TimeCounter countdownTimer = gameObject.GetComponent<TimeCounter>();      // Then, access the Script in the GameObject
        //countdownTimer.startTimer(countdownLength);
        Invoke("SettingActive", TimeToWait);// Finally, call the Script method                                                      
        //CancelInvoke();
        //Invoke("RestartGame", 1);
    }
   
    public void SettingActive()
    {
        //Debug.Log("SettingActive");
        if (IsHovering == true)
            {
            Obj.SetActive(true);
            depthBacking.SetActive(true);
        }
        else
        {
            CancelInvoke();
        }
    }
}
