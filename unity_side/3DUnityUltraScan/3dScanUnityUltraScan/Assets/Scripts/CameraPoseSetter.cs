using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraPoseSetter : MonoBehaviour
{
    public UDP Udp;
    public int index;
    public Color color;

    private Vector3 point;
	
	// Update is called once per frame
	void Update ()
    {
        var t = Udp.CamData[index];

        if (t.position.x > 0)
        {
            transform.position = t.position;
            transform.rotation = t.rotation;
        }

        point = t.lightDotPos;
        point.z = 1;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, transform.TransformDirection(-point) * 50);
    }
}
