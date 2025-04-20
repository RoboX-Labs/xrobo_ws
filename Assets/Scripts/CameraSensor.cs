using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    private Camera cam; // Camera reference

    void Start()
    {
        // Get the Camera component attached to this GameObject
        cam = GetComponent<Camera>();

        // Check if the camera is assigned
        if (cam == null)
        {
            Debug.LogError("Camera not assigned. Please assign a Camera component to this GameObject.");
            return;
        }

        
    }

    public float GetDistanceAtPixel(int pixelX, int pixelY)
    {
        if (cam == null)
        {
            Debug.LogError("Camera not assigned.");
            return -1f; // Return -1 if no camera is assigned
        }

        // Perform a raycast from the given pixel position
        Ray ray = cam.ScreenPointToRay(new Vector3(pixelX, pixelY, 0));
        RaycastHit hit;

        // Visualize the raycast in the Scene view
        Debug.DrawRay(ray.origin, ray.direction * 100f, Color.red, 0.1f);

        if (Physics.Raycast(ray, out hit))
        {
            // Visualize the hit point
            Debug.DrawRay(ray.origin, ray.direction * hit.distance, Color.green, 0.1f);

            // Return the distance to the hit object
            return Vector3.Distance(cam.transform.position, hit.point);
        }

        // Return -1 if no object is hit
        return -1f;
    }

    void OnGUI()
    {
        if (cam == null) return; // Exit if no Camera is assigned

        // Example: Get the distance at the center of the screen
        int centerX = cam.pixelWidth / 2;
        int centerY = cam.pixelHeight / 2;
        float distance = GetDistanceAtPixel(centerX, centerY);

        // Display the information on the game window
        GUILayout.BeginArea(new Rect(20, 20, 300, 100));
        GUILayout.Label("Center pixel: (" + centerX + ", " + centerY + ")");
        GUILayout.Label("Distance to camera: " + (distance >= 0 ? distance.ToString("F3") + " units" : "No object hit"));
        GUILayout.EndArea();
    }
}