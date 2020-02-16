using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointCloudCreator : MonoBehaviour
{
    public UDP Udp;

	// Use this for initialization
	void Start () {

    }

    private Vector3 cam1Pos;
    private Vector3 cam2Pos;
    private Quaternion cam1Rotation;
    private Quaternion cam2Rotation;

    private readonly List<Vector3> pointCloud = new List<Vector3>();

    private int lastMessageId;

    private Ray cam1Ray;
    private Ray cam2Ray;

    Vector3 findRayIntersection(Ray cam1Ray, Ray cam2Ray)
    {
        var a = cam1Ray.direction.normalized;
        var b = cam2Ray.direction.normalized;
        var c = cam2Ray.origin - cam1Ray.origin;

        var D = cam1Ray.origin + a * ((-Vector3.Dot(a, b) * Vector3.Dot(b, c) + Vector3.Dot(a, c) * Vector3.Dot(b, b)) /
                                       (Vector3.Dot(a, a) * Vector3.Dot(b, b) - Vector3.Dot(a, b) * Vector3.Dot(a, b)));
        var E = cam2Ray.origin + b * ((Vector3.Dot(a, b) * Vector3.Dot(a, c) - Vector3.Dot(b, c) * Vector3.Dot(a, a)) /
                                       (Vector3.Dot(a, a) * Vector3.Dot(b, b) - Vector3.Dot(a, b) * Vector3.Dot(a, b)));

        return (D+E)/2;
    }
	
	// Update is called once per frame
	void Update () {
        var camData1 = Udp.CamData[0];
        var camData2 = Udp.CamData[1];

        if (camData1.position.x > 0 && camData2.position.y > 0)
        {
            cam1Pos = camData1.position;
            cam2Pos = camData2.position;
            cam1Rotation = camData1.rotation;
            cam2Rotation = camData2.rotation;
        }

        if (camData1.lightDotDiretion.x != 0 && camData2.lightDotDiretion.x != 0 && lastMessageId < Udp.GetIndexOfLastMessage())
        {
            lastMessageId = Udp.GetIndexOfLastMessage();
            Vector3 cam1LightDir = -camData1.lightDotDiretion;
            cam1LightDir.z = -1;
            Vector3 cam2LightDir = -camData2.lightDotDiretion;
            cam2LightDir.z = -1;

            cam1Ray = new Ray(cam1Pos, cam1Rotation * cam1LightDir);
            cam2Ray = new Ray(cam2Pos, cam2Rotation * cam2LightDir);

            pointCloud.Add(findRayIntersection(cam1Ray, cam2Ray));
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            pointCloud.Clear();
        }
	}

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        foreach (var point in pointCloud)
        {
            Gizmos.DrawSphere(point, 0.1f);
        }

        Gizmos.color = Color.yellow;
        
        if (cam1Ray.origin.x != 0)
        {
            Gizmos.DrawRay(cam1Ray.origin, cam1Ray.direction * 50);
            Gizmos.DrawRay(cam2Ray.origin, cam2Ray.direction * 50);
        }
    }
}
