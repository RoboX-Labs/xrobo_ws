using UnityEngine;

public class BasketballController : MonoBehaviour
{
    private ScoreCounter scoreCounter;

    private void Start()
    {
        scoreCounter = FindFirstObjectByType<ScoreCounter>();
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.name == "Body9")
        {
            Debug.Log("Basketball out of zone! Destroying...");
            // Destroy(this.gameObject);
        }
    }
}
