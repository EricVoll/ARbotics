using UnityEngine;
using UnityEngine.UI;


public class Change2DLidarProperties : MonoBehaviour
{
    // Start is called before the first frame update
    public Slider slider;
    void Start()
    {
        //Start the Camera field of view at 60
        if (slider == null) slider = GetComponent<Slider>();
        if (gameObject.tag == "AngleOfVision")
        {
            VisionCone.angleOfVision = slider.value * (Mathf.PI / 180.0f);
        }


        if (gameObject.tag == "SightDepth")
        {
            VisionCone.sightRange = (int)slider.value;
        }

        //IsPhysicalProp = Cam.usePhysicalProperties;
    }
    public void ChangeVisOrDepth(Slider slider)
    {
        //Cam = gameObject.GetComponent<Camera>();
        //Set up the maximum and minimum values the Slider can return (you can change these)
        //max = 150.0f;
        //min = 20.0f;
        //Debug.Log("ChangeFov");
        if (gameObject.tag == "AngleOfVision")
        {
            VisionCone.angleOfVision = slider.value * ( Mathf.PI / 180.0f);
        }


        if (gameObject.tag == "SightDepth")
        {
            VisionCone.sightRange = (int)slider.value;
        }
        //This Slider changes the field of view of the Camera between the minimum and maximum values
        //m_FieldOfView = GUI.HorizontalSlider(new Rect(20, 20, 100, 40), m_FieldOfView, min, max);
    }
}
