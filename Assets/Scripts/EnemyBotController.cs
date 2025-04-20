using UnityEngine;

public class EnemyBotController : MonoBehaviour
{
    public Transform target; // The target object to move towards
    public float speed = 5f; // Speed of the bot
    public float stopDistance = 2f; // Distance to maintain from the target

    // Update is called once per frame
    void FixedUpdate()
    {
        if (target == null) return;

        // Calculate the distance to the target
        float distanceToTarget = Vector3.Distance(transform.position, target.position);

        // Stop moving if within the desired distance
        if (distanceToTarget <= stopDistance) return;

        // Calculate the direction towards the target
        Vector3 direction = (target.position - transform.position).normalized;

        // Move the bot towards the target
        transform.position += direction * speed * Time.deltaTime;
    }
}