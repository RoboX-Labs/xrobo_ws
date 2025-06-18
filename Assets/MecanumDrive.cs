using UnityEngine;

public struct Mecanum
{
    public float lx;
    public float ly;
    public float r;

    public Mecanum(float lx, float ly, float r)
    {
        this.lx = lx;
        this.ly = ly;
        this.r = r;
    }

    /// <summary>
    /// Calculates angular velocities for each wheel
    /// Return type : float[4] (radians per second rad/s)
    /// </summary>
    public float[] GetAngularVelocities(float vx, float vy, float omega)
    {
        float[] angularVelocities = new float[4];
        float sum = this.lx + this.ly;
        angularVelocities[0] = (vx - vy - sum * omega) / this.r; // FL
        angularVelocities[1] = (vx + vy + sum * omega) / this.r; // FR
        angularVelocities[2] = (vx + vy - sum * omega) / this.r; // BL
        angularVelocities[3] = (vx - vy + sum * omega) / this.r; // BR
        return angularVelocities;
    }
}

public class MecanumDrive : MonoBehaviour
{
    public Mecanum mecanum;
    public HingeJoint[] wheels = new HingeJoint[4];
    public JointMotor[] motors = new JointMotor[4];
    public float moveSpeed = 0.001f;
    public float maxWheelSpeed = 100f;
    public float motorForce = 100f;

    private void Start()
    {
        mecanum = new Mecanum(0.25f, 0.25f, 0.1f);
        for (int i = 0; i < motors.Length; i++)
        {
            motors[i] = new JointMotor();
        }
    }

    private void FixedUpdate()
    {
        float moveX = Input.GetAxisRaw("Horizontal");  // Range: -1 to 1
        float moveZ = Input.GetAxisRaw("Vertical");

        float[] wheelVels = mecanum.GetAngularVelocities(
            moveX * moveSpeed,
            moveZ * moveSpeed,
            0f
        );

        Debug.Log($"Wheel Velocities: {string.Join(", ", wheelVels)}");

        for (int i = 0; i < wheels.Length; i++)
        {
            motors[i].force = motorForce;
            motors[i].targetVelocity = i % 2 != 0 ? wheelVels[i] : -wheelVels[i];
            motors[i].targetVelocity = Mathf.Rad2Deg * motors[i].targetVelocity;
            motors[i].targetVelocity = Mathf.Clamp(motors[i].targetVelocity, -maxWheelSpeed, maxWheelSpeed);
            wheels[i].motor = motors[i];
        }
    }
}
