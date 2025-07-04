using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Joy = RosMessageTypes.Sensor.JoyMsg;

public class PlayerNode : MonoBehaviour
{
    PlayerController player;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Joy>("joy", JoyChange);
        player = GetComponent<PlayerController>();
        player.useSim = true;
    }

    void JoyChange(Joy joyMessage)
    {
        // Debug.Log($"Joy axes: {joyMessage.axes[0]}, {joyMessage.axes[1]}");
        player.Move(new Vector3(-joyMessage.axes[1], joyMessage.axes[3], joyMessage.axes[0]), new Vector3(0, joyMessage.axes[2], 0));

        if (joyMessage.buttons[0] == 1)
        {
            player.Jump();
        }

        if (joyMessage.buttons[1] == 1)
        {
            player.Shoot();
        }
    }
}