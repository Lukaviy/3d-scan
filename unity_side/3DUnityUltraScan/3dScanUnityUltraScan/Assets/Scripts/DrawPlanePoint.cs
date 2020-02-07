using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawPlanePoint : MonoBehaviour
{
    public GameObject PointPrefab;

	void Start () {
        for (var i = 0; i < 9; i++)
        {
            for (var j = 0; j < 6; j++)
            {
                Instantiate(PointPrefab, new Vector3(i, j, 0), Quaternion.identity);
            }
        }
	}
	
	void Update () {
		
	}

    void OnDrawGizmos()
    {
        
    }
}
