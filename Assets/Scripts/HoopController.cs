using UnityEngine;

public class HoopController : MonoBehaviour
{
    private ScoreCounter scoreCounter;

    private void Start()
    {
        scoreCounter = FindFirstObjectByType<ScoreCounter>();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.name == "Basketball(Clone)")
        {
            scoreCounter.IncrementScore(1);
            Debug.Log($"Basketball in score zone! {scoreCounter.score}");
        }
    }
}
