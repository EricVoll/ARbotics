using UnityEngine;
using UnityEngine.UI;


public class SpecialTriggerAdvanced : MonoBehaviour
{
    //// Start is called before the first frame update
    public Toggle toggleFrustum;



    public void nced()
    {
        //GLDebug.triggerOfDisp = !GLDebug.triggerOfDisp;


        CameraMesh.triggerOfDispAdvanced = !CameraMesh.triggerOfDispAdvanced;

        if (CameraFrustumInGame.triggerOfDispSimple)
        {
            toggleFrustum.isOn = false; 
            CameraFrustumInGame.triggerOfDispSimple = !CameraFrustumInGame.triggerOfDispSimple;
        }
    }
}
