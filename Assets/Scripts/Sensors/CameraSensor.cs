#define DEBUG 

using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
    }

    public float GetDistanceAtPixel(int pixelX, int pixelY)
    {
        Ray ray = cam.ScreenPointToRay(new Vector3(pixelX, pixelY, 0));
        RaycastHit hit;

#if (DEBUG)
        Debug.DrawRay(ray.origin, ray.direction * 100f, Color.red, 0.1f);
#endif

        if (Physics.Raycast(ray, out hit))
        {
#if (DEBUG)

            Debug.DrawRay(ray.origin, ray.direction * hit.distance, Color.green, 0.1f);
#endif
            return Vector3.Distance(cam.transform.position, hit.point);
        }

        return -1f;
    }

    void OnGUI()
    {
        int centerX = cam.pixelWidth / 2;
        int centerY = cam.pixelHeight / 2;
        float distance = GetDistanceAtPixel(centerX, centerY);

        GUILayout.BeginArea(new Rect(20, 20, 300, 100));
        GUILayout.Label("Center pixel: (" + centerX + ", " + centerY + ")");
        GUILayout.Label("Distance to camera: " + (distance >= 0 ? distance.ToString("F3") + " units" : "No object hit"));
        GUILayout.EndArea();
    }
}