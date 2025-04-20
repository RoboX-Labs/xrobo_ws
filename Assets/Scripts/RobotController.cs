using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;
    public float jumpForce = 5f;

    private Rigidbody rb;
    private LaunchProjectile launchProjectile;
    private bool isHoldingBall = true;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        launchProjectile = GetComponent<LaunchProjectile>();

        if (rb == null)
        {
            Debug.LogError("Rigidbody component is missing!");
        }
    }

    void Update()
    {
        HandleMovement();
        HandleRotation();
        HandleJump();
        HandleShooting();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name == "Basketball(Clone)")
        {
            Debug.Log("Basketball collision detected!");
        }

        if (collision.gameObject.name == "EnemyRobot")
        {
            Debug.Log("Enemy robot collision detected! Relative velocity: " + collision.relativeVelocity.magnitude);
        }
    }

    private void HandleShooting()
    {
        if (isHoldingBall)
        {
            launchProjectile.ShootBall();
        }
    }

    private void HandleMovement()
    {
        float moveZ = Input.GetAxis("Vertical") * moveSpeed * Time.deltaTime;
        float moveX = Input.GetAxis("Horizontal") * moveSpeed * Time.deltaTime;
        transform.Translate(moveX, 0, moveZ);
    }

    private void HandleRotation()
    {
        float rotation = 0f;

        if (Input.GetKey(KeyCode.J))
        {
            rotation = -rotationSpeed * Time.deltaTime;
        }
        else if (Input.GetKey(KeyCode.L))
        {
            rotation = rotationSpeed * Time.deltaTime;
        }

        transform.Rotate(0, rotation, 0);
    }

    private void HandleJump()
    {
        if (Input.GetKeyDown(KeyCode.K) && IsGrounded())
        {
            rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
            Debug.Log("Jumping");
        }
    }

    private bool IsGrounded()
    {
        return Physics.Raycast(transform.position, Vector3.down, 1f);
    }
}