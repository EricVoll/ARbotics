using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestScript : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        try
        {

            SomeClass t = JsonUtility.FromJson<SomeClass>("{\"test\":\"This test\"}");
            if (t.test == "This test")
            {
                this.transform.Find("Cube").gameObject.SetActive(true);
            }
        }
        catch
        {

        }

        try
        {

            SomeClass t = Newtonsoft.Json.JsonConvert.DeserializeObject<SomeClass>("{\"test\":\"This test\"}");
            if (t.test == "This test")
            {
                this.transform.Find("TestCube").gameObject.SetActive(true);
            }
        }
        catch
        {

        }
    }

    class SomeClass
    {
        public string test = "";
    }
}
