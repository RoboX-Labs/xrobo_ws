using UnityEngine;


public class PlayerController : MonoBehaviour
{
    public Rigidbody m_Rigidbody;
    public float m_Speed = 15f;
    public float m_RotationSpeed = 100f;
    public float m_Thrust = 200f;
    public bool useSim = false;

    [Header("Camera")]
    public Transform m_CameraTransform;

    void Start()
    {
        m_Rigidbody = GetComponent<Rigidbody>();
    }
    void FixedUpdate()
    {
        if (!useSim)
        {
            ManualMovement();

        }
    }
    public void ManualMovement()
    {
        Vector3 m_Input = new Vector3(Input.GetAxis("Horizontal"), 0, Input.GetAxis("Vertical"));
        Vector3 m_EulerAngleVelocity = new Vector3(0, Input.GetAxis("Mouse X") * 100f, 0);
        Move(m_Input, m_EulerAngleVelocity);

        if (Input.anyKeyDown)
        {
            switch (true)
            {
                case bool _ when Input.GetKeyDown(KeyCode.Alpha1):
                    Jump();
                    break;
                case bool _ when Input.GetKeyDown(KeyCode.Alpha2):
                    Dribble();
                    break;
                case bool _ when Input.GetKeyDown(KeyCode.Alpha3):
                    Shoot();
                    break;

            }
        }
    }

    public void Move(Vector3 m_Input, Vector3 m_EulerAngleVelocity)
    {
        Debug.Log($"Input: {m_Input}, EulerAngleVelocity: {m_EulerAngleVelocity}");
        m_Rigidbody.MovePosition(transform.position + m_Input * Time.fixedDeltaTime * m_Speed);

        if (!useSim) m_EulerAngleVelocity *= m_RotationSpeed;

        Quaternion deltaRotation = Quaternion.Euler(m_EulerAngleVelocity * Time.fixedDeltaTime);
        m_Rigidbody.MoveRotation(m_Rigidbody.rotation * deltaRotation);
    }
    public void Jump()
    {
        Debug.Log("Jump action triggered 🚀");
        if (m_Rigidbody.linearVelocity.y == 0)
        {
            m_Rigidbody.AddForce(transform.up * m_Thrust);
        }
    }

    public void Dunk()
    {
        Debug.Log("Dunk action triggered 🏀");
        // if (m_Rigidbody.linearVelocity.y == 0)
        // {
        //     m_Rigidbody.AddForce(transform.up * m_Thrust * 2);
        // }
    }

    public void Dribble()
    {
        Debug.Log("Dribble action triggered 🏀");
    }

    public void Shoot()
    {
        Debug.Log("Shoot action triggered 🎯");
    }

    public void Pass()
    {
        Debug.Log("Pass action triggered 🏃‍♂️");
    }

    public void MovePanTiltCamera(Vector3 panTilt)
    {
        m_CameraTransform.Rotate(panTilt);
    }
}
