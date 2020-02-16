using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraPoseSetter : MonoBehaviour
{
    public UDP Udp;
    public int index;
    public Color color;

    private Vector3 dotDirection;
	
	// Update is called once per frame
	void Update ()
    {
        var t = Udp.CamData[index];

        if (t.position.x != 0)
        {
            transform.position = t.position;
            transform.rotation = t.rotation;
        }

        dotDirection = t.lightDotDiretion;
        dotDirection.z = 1;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        if (dotDirection.x != 0)
        {
           // Gizmos.DrawRay(transform.position, transform.TransformDirection(-dotDirection) * 50);
        }
    }
}
