using UnityEngine;
using UnityEngine.UI; // Required if you want to display the timer on a UI Text element
using System.Collections;

public class TimerCounter : MonoBehaviour
{
    public float timeRemaining = 20f; // Countdown starts from 20 seconds
    public bool timerIsRunning = false; // Tracks if the timer is running
    public Text scoreCounterText;
    public Text timerText; // Optional: Assign a UI Text element to display the timer
    public Text countdownText; // Assign a UI Text element for the countdown (3, 2, 1)

    void Start()
    {
        if (countdownText != null)
        {
            StartCoroutine(StartCountdown());
        }
        else
        {
            timerIsRunning = true; // Start the timer immediately if no countdown text is assigned
        }
    }

    void Update()
    {
        if (timerIsRunning)
        {
            if (timeRemaining > 0)
            {
                timeRemaining -= Time.deltaTime; // Decrease time
                DisplayTime(timeRemaining); // Optional: Update the UI
            }
            else
            {
                timeRemaining = 0; // Ensure it doesn't go below 0
                timerIsRunning = false; // Stop the timer
                DisplayTime(timeRemaining); // Update the UI to reflect the end of the timer
                Debug.Log("Timer has ended!");
            }
        }
    }

    // Coroutine for the countdown (3, 2, 1)
    private IEnumerator StartCountdown()
    {
        scoreCounterText.gameObject.SetActive(false); // Hide the score counter text
        timerText.gameObject.SetActive(false); // Hide the timer text
        countdownText.gameObject.SetActive(true); // Show the countdown text


        for (int i = 3; i > 0; i--)
        {
            countdownText.text = i.ToString(); // Display the countdown number
            yield return new WaitForSeconds(1f); // Wait for 1 second
        }

        countdownText.gameObject.SetActive(false); // Hide the countdown text
        scoreCounterText.gameObject.SetActive(true); // Show the score counter text
        timerText.gameObject.SetActive(true); // Show the timer text
        timerIsRunning = true; // Start the timer
    }

    // Optional: Display the time in a UI Text element
    void DisplayTime(float timeToDisplay)
    {
        if (timerText != null)
        {
            if (timeToDisplay <= 0)
            {
                timerText.text = "End Game"; // Display "End Game" when the timer ends
                timerText.color = Color.red; // Set the text color to red
            }
            else
            {
                timeToDisplay = Mathf.Max(0, timeToDisplay); // Ensure no negative time
                int minutes = Mathf.FloorToInt(timeToDisplay / 60); // Calculate minutes
                int seconds = Mathf.FloorToInt(timeToDisplay % 60); // Calculate seconds
                timerText.text = string.Format("{0:00}:{1:00}", minutes, seconds); // Format as MM:SS

                // Change the text color when time is less than 5 seconds
                if (timeToDisplay < 5f)
                {
                    timerText.color = Color.red; // Set the text color to red
                }
                else
                {
                    timerText.color = Color.white; // Reset the text color to white
                }
            }
        }
    }
}