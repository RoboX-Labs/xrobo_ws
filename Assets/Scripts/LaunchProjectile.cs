using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class LaunchProjectile : MonoBehaviour
{
    [Header("Projectile Settings")]
    public GameObject basketballPrefab;
    public Transform posShoot;
    public Transform destination;
    public LineRenderer trajectoryLine;
    [SerializeField] private float initialVelocity = 15f;
    [SerializeField] private int trajectoryResolution = 50;

    private const float Gravity = 9.81f;
    private bool isDragging = false;
    private GameObject basketball;

    private void Start()
    {
        ValidateComponents();
        InitializeBasketball();
    }

    public void ShootBall()
    {
        if (Input.GetMouseButtonDown(0)) isDragging = true;
        if (Input.GetMouseButton(0) && isDragging) { AdjustDestinationWithMouse(); DrawTrajectory(); }
        if (Input.GetMouseButtonUp(0)) { isDragging = false; trajectoryLine.positionCount = 0; Launch(); }
        if (Input.GetMouseButtonDown(1)) { isDragging = false; trajectoryLine.positionCount = 0; }
    }

    private void Launch()
    {
        if (basketball == null) return;

        Vector3 shootPos = posShoot.position;
        Vector3 targetPos = AdjustTargetHeight(destination.position);

        Vector3 launchVelocity = CalculateLaunchVelocity(shootPos, targetPos);
        if (launchVelocity == Vector3.zero) return;

        ActivateBasketball(shootPos, launchVelocity);
    }

    private void AdjustDestinationWithMouse()
    {
        if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out RaycastHit hit))
            destination.position = hit.point;
    }

    private void DrawTrajectory()
    {
        if (trajectoryLine == null || posShoot == null || destination == null) return;

        Vector3 shootPos = posShoot.position;
        Vector3 targetPos = AdjustTargetHeight(destination.position);

        Vector3 launchVelocity = CalculateLaunchVelocity(shootPos, targetPos);
        if (launchVelocity == Vector3.zero) { trajectoryLine.positionCount = 0; return; }

        trajectoryLine.positionCount = trajectoryResolution;
        for (int i = 0; i < trajectoryResolution; i++)
        {
            float t = i / (float)(trajectoryResolution - 1) * (2 * launchVelocity.magnitude / Gravity);
            Vector3 point = shootPos + launchVelocity * t + 0.5f * Physics.gravity * t * t;
            trajectoryLine.SetPosition(i, point);
        }
    }

    private Vector3 CalculateLaunchVelocity(Vector3 shootPos, Vector3 targetPos)
    {
        Vector3 delta = targetPos - shootPos;
        float x = new Vector2(delta.x, delta.z).magnitude;
        float y = delta.y;

        float v0Squared = initialVelocity * initialVelocity;
        float discriminant = v0Squared * v0Squared - Gravity * (Gravity * x * x + 2 * y * v0Squared);

        if (discriminant < 0) return Vector3.zero;

        float angle = Mathf.Atan((v0Squared + Mathf.Sqrt(discriminant)) / (Gravity * x));
        Vector3 flatDir = new Vector3(delta.x, 0, delta.z).normalized;

        return Quaternion.AngleAxis(Mathf.Rad2Deg * angle, Vector3.Cross(flatDir, Vector3.up)) * flatDir * initialVelocity;
    }

    private Vector3 AdjustTargetHeight(Vector3 targetPos)
    {
        targetPos.y = 3.05f; // Adjust to hoop height
        return targetPos;
    }

    private void ActivateBasketball(Vector3 position, Vector3 velocity)
    {
        basketball.transform.position = position;
        basketball.SetActive(true);

        Rigidbody rb = basketball.GetComponent<Rigidbody>();
        rb.linearVelocity = velocity;
        rb.angularVelocity = Vector3.zero;
    }

    private void ValidateComponents()
    {
        if (basketballPrefab == null) Debug.LogError("Basketball prefab not assigned.");
        if (posShoot == null) Debug.LogError("posShoot not assigned.");
        if (destination == null) Debug.LogError("Destination not assigned.");
        if (trajectoryLine == null) Debug.LogError("Trajectory LineRenderer not assigned.");
    }

    private void InitializeBasketball()
    {
        if (basketballPrefab != null)
        {
            basketball = Instantiate(basketballPrefab, posShoot.position, Quaternion.identity);
            basketball.SetActive(false);
        }
    }
}