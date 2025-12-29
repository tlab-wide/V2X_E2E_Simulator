using System.Collections.Generic;
using AWSIM;
using UnityEngine;

public class SelectFromTheList : MonoBehaviour
{
    [SerializeField]private List<GameObject> objects;

    public void ActivateElement(int index)
    {
        for (int i = 0; i < objects.Count; i++)
        {
            if (i == index)
            {
                objects[i].SetActive(true);
            }
            else
            {
                objects[i].SetActive(false);
            }
        }
        
    }
}
