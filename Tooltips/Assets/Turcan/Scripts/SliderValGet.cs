using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class SliderValGet : MonoBehaviour
{

    public float input;
    public Slider slider;

    void Start()
    {
        if (slider == null) slider = GetComponent<Slider>(); // search slider in this object if its not set in unity inspector
    }

    public void printValue(Slider slider)
    {

        input=slider.value;
        Debug.Log(input);

    }

}