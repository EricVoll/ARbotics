//Attach this script to an empty GameObject.
//This script creates a Slider that allows you to manipulate the Camera's field of view. Place GameObjects in the Scene to show the full effect.

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;

public class CameraPropertiesChange : MonoBehaviour
{
    //This is the field of view that the Camera has
    //readonly float m_FieldOfView;
    readonly float InitialDepth=0.0f;
    public Camera Cam;
    public Slider slider;
    public Slider FOVslider;
    public Toggle toggle;
    public Dropdown dropdown;
    int filmHeight = 24;
    int filmWidth = 36;
    public static bool IsPhysicalProp;
    bool tog;
    string DropDownCaption;
    void Start()
    {
        //Start the Camera field of view at 60
        if (slider == null) slider = GetComponent<Slider>();
        //Cam.fieldOfView = m_FieldOfView;
        Cam.depth = InitialDepth;
        //IsPhysicalProp = Cam.usePhysicalProperties;
    }

    //void Update()
    //{
    //    //Update the camera's field of view to be the variable returning from the Slider
    //    Camera.main.fieldOfView = m_FieldOfView;
    //}

    public void ChangeFovOrDepth(Slider slider)
    {
        //Cam = gameObject.GetComponent<Camera>();
        //Set up the maximum and minimum values the Slider can return (you can change these)
        //max = 150.0f;
        //min = 20.0f;
        //Debug.Log("ChangeFov");
        if (gameObject.tag == "FoV")
        {
            Cam.fieldOfView = slider.value;
        }
        
        if (gameObject.tag=="Depth")
        {
            Cam.depth = slider.value;
        }

        if (gameObject.tag == "NearClipSlider")
        {
            Cam.nearClipPlane = slider.value;
        }

        if (gameObject.tag == "FarClipSlider")
        {
            Cam.farClipPlane = slider.value;
        }

        if (gameObject.tag == "FocalLength" && Cam.usePhysicalProperties == true)
        {
            Cam.focalLength = slider.value;
            double fovdub = Mathf.Rad2Deg * 2.0 * Math.Atan(filmHeight / (2.0 * slider.value));
            FOVslider.value = (float)fovdub;

        }


        //if (gameObject.tag == " ")
        //{
        //    Cam.depth = slider.value;
        //}
        //This Slider changes the field of view of the Camera between the minimum and maximum values
        //m_FieldOfView = GUI.HorizontalSlider(new Rect(20, 20, 100, 40), m_FieldOfView, min, max);
    }
    public void ChangeToogle(Toggle toggle)
    {
        //Cam = gameObject.GetComponent<Camera>();
        //Set up the maximum and minimum values the Slider can return (you can change these)
        //max = 150.0f;
        //min = 20.0f;

        //Debug.Log(tog);
        //Debug.Log("ChangeFov");
        if (gameObject.tag == "PhysicalProperties")
        {
            tog = toggle.isOn;
            if (tog == true)
            {
                //Debug.Log("ToggleFalse");
                
                Cam.usePhysicalProperties = true;
                IsPhysicalProp = true;

                //Slider.Find("FocalLengthSlider").SetActive(true);
                //Dropdown.Find("Physical_SensorTypeDropDown").SetActive(true);

            }
            else if (tog == false)
            {

                Cam.usePhysicalProperties = false;
                IsPhysicalProp = false;

                //GameObject.FindObjectOfType(typeof(Slider));
                //("FocalLengthSlider").SetActive(false);
                //GameObject.Find("Physical_SensorTypeDropDown").SetActive(false);

            }


        }

        if (gameObject.tag == "Occlusion")
        {
            //Debug.Log(gameObject.tag);
            tog = toggle.isOn;
            Debug.Log(Cam.useOcclusionCulling.ToString());
            if (tog == true & Cam.useOcclusionCulling==false)
            {


                Cam.useOcclusionCulling = true;
                //IsPhysicalProp = true;

                //Slider.Find("FocalLengthSlider").SetActive(true);
                //Dropdown.Find("Physical_SensorTypeDropDown").SetActive(true);

            }
            else if (tog == false & Cam.useOcclusionCulling == true)
            {
                //Debug.Log("ToggleFalse");
                Cam.useOcclusionCulling = false;
                //IsPhysicalProp = false;

                //GameObject.FindObjectOfType(typeof(Slider));
                //("FocalLengthSlider").SetActive(false);
                //GameObject.Find("Physical_SensorTypeDropDown").SetActive(false);

            }


        }
        //This Slider changes the field of view of the Camera between the minimum and maximum values
        //m_FieldOfView = GUI.HorizontalSlider(new Rect(20, 20, 100, 40), m_FieldOfView, min, max);
    }
    public void ChangeDropdown(Dropdown dropdown)
    {
        //Cam = gameObject.GetComponent<Camera>();
        //Set up the maximum and minimum values the Slider can return (you can change these)
        //max = 150.0f;
        //min = 20.0f;
        //Debug.Log("ChangeFov");
        if (gameObject.tag == "FoVAxis")
        {

        }


        if (gameObject.tag == "Projection")
        {

            DropDownCaption = EventSystem.current.currentSelectedGameObject.name;
            Debug.Log(DropDownCaption);
            if (DropDownCaption == "Item 0: Perspective")
            {
                Cam.orthographic = false;
            }
            else if (DropDownCaption == "Item 1: Orthographic")
            {
                Cam.orthographic = true;
            }
        }
        //This Slider changes the field of view of the Camera between the minimum and maximum values
        //m_FieldOfView = GUI.HorizontalSlider(new Rect(20, 20, 100, 40), m_FieldOfView, min, max);
    }
}