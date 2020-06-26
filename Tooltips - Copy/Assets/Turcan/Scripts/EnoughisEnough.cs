using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EnoughisEnough : MonoBehaviour
{
    public static bool hasParent = false;
    public GameObject Child;
    private GameObject Parent2;

    public void OnTriggerEnter(Collider other)
    {

        //if (other.tag == "Robot") //ther.tag == gameObject.tag
        //{
            //Debug.Log("Other Tag= Robot");
            if (hasParent == false)
            {
                //print(hasParent);
                //Parent2 = GameObject.FindWithTag("Robot");
                Parent2 = FindClosestRobot();
                //Child.transform.localPosition=new Vector3(0.00f, 0.00f, 0.00f);
                Child.transform.SetParent(Parent2.transform, true);
            }
            //istriggered = true;
            //tempTrans = Child.transform.parent;
            //void Example(Transform newParent, GameObject child)
            //{
            hasParent = true;

    }
    public GameObject FindClosestRobot()
    {
        GameObject[] gos;
        gos = GameObject.FindGameObjectsWithTag("Robot");
        GameObject closest = null;
        float distance = Mathf.Infinity;
        Vector3 position = transform.position;
        foreach (GameObject go in gos)
        {
            Vector3 diff = go.transform.position - position;
            float curDistance = diff.sqrMagnitude;
            if (curDistance < distance)
            {
                closest = go;
                distance = curDistance;
            }
        }
        //Debug.Log("FoundTheClosest");
        return closest;
    }
}
