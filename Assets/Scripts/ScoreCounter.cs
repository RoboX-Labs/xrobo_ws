using UnityEngine;
using UnityEngine.UI; // Required for UI Text

public class ScoreCounter : MonoBehaviour
{
    public int score = 0;
    public Text scoreText;

    void Start()
    {
        // Initialize the score to 0 at the start of the game
        score = 0;
        UpdateScoreText(); // Update the UI
        Debug.Log("ScoreCounter initialized. Score reset to 0.");
    }

    public void IncrementScore(int points = 1)
    {
        // Increment the score by the specified number of points
        score += points;
        UpdateScoreText(); // Update the UI
    }

    public void ResetScore()
    {
        score = 0;
        UpdateScoreText(); // Update the UI
    }

    private void UpdateScoreText()
    {
        if (scoreText != null)
        {
            scoreText.text = "Score: " + score; // Update the text to display the current score
        }
        else
        {
            Debug.LogWarning("Score Text UI element is not assigned!");
        }
    }
}