using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TriggerExit : MonoBehaviour
{
    public float delay = 5f;
    
    public delegate void ExitAction();
    public static event ExitAction OnChunkExited;

    private bool exited = false;

    void OnTriggerEnter(Collider other)
    {
        CarTag carTag = other.GetComponent<CarTag>();
        if (carTag != null)
        {
            if (!exited)
            {
                exited = true;
                OnChunkExited();
                StartCoroutine(WaitAndDeactivate());
                Debug.Log("working");
            }


        }
    }

    IEnumerator WaitAndDeactivate()
    {
        yield return new WaitForSeconds(delay);

        transform.root.gameObject.SetActive(false);

    }



}
