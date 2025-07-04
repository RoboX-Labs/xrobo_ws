using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LidarSensor : MonoBehaviour
{
    public int NumberOfIncrements = 360;
    public static float MaxRange = 15f;

    [HideInInspector]
    public static float[] distances;
 
    void Start () {
        distances = new float[NumberOfIncrements];
    }

    void FixedUpdate () {
        Vector3 fwd = new Vector3(0, 0, 1);
        Vector3 dir;
        RaycastHit hit;
        int indx = 0;

        for (int incr = 0; incr < NumberOfIncrements; incr++)
        {
            indx = incr;
            dir = transform.rotation * Quaternion.Euler(0, incr, 0)*fwd;
            
            if (Physics.Raycast(transform.position, dir, out hit, MaxRange))
            {
                distances[indx] = (float)hit.distance;
            }
            else
            {
                distances[indx] = MaxRange;
            }
            Debug.DrawRay(transform.position, dir * distances[indx], Color.red);
        }
        // Debug.Log("Lidar distances updated: " + string.Join(", ", distances));
    }
}