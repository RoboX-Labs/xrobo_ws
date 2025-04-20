using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class CameraNode : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    public string topicName = "camera/image_raw";

    [SerializeField]
    public float publishMessageFrequency = 0.1f; // 10 Hz (adjust as needed)

    [SerializeField]
    public Camera inputCamImage;

    private Texture2D reusableTexture;
    private float timeElapsed = 0f;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Initialize reusable texture
        int width = inputCamImage.targetTexture.width;
        int height = inputCamImage.targetTexture.height;
        reusableTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void OnDestroy()
    {
        // Clean up texture when script is destroyed
        if (reusableTexture != null)
        {
            Destroy(reusableTexture);
        }
    }

    void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishMessageFrequency)
        {
            SendImage();
            timeElapsed = 0f;
        }
    }

    void SendImage()
    {
        // Render the camera manually to the target texture
        var oldRT = RenderTexture.active;
        RenderTexture.active = inputCamImage.targetTexture;

        inputCamImage.Render();

        // Read pixels into reusable texture
        reusableTexture.ReadPixels(new Rect(0, 0, reusableTexture.width, reusableTexture.height), 0, 0);
        reusableTexture.Apply();

        RenderTexture.active = oldRT;

        // Convert to ROS message and publish
        HeaderMsg header = new HeaderMsg();
        ImageMsg imageMsg = reusableTexture.ToImageMsg(header);
        ros.Publish(topicName, imageMsg);
    }
}
