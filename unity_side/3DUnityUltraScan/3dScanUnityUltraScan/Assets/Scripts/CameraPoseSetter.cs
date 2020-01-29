using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraPoseSetter : MonoBehaviour
{
    public UDP Udp;
    public int index;
    public Color color;
	
	// Update is called once per frame
	void Update ()
    {
        var t = Udp.CamData[index];

        transform.position = t.position;
        transform.rotation = t.rotation;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, -transform.TransformDirection(Vector3.forward) * 50);
    }
}
