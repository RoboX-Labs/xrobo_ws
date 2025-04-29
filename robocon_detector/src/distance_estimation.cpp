#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Distance from camera to object (face) measured in centimeters
const float Known_distance = 76.2;

// Width of face in the real world or Object Plane in centimeters
const float Known_width = 14.3;

// Colors
const Scalar GREEN = Scalar(0, 255, 0);
const Scalar RED = Scalar(0, 0, 255);
const Scalar WHITE = Scalar(255, 255, 255);
const Scalar BLACK = Scalar(0, 0, 0);

// Define the fonts
const int fonts = FONT_HERSHEY_COMPLEX;

// Create the face detector object
CascadeClassifier face_detector;

// Focal length finder function
float Focal_Length_Finder(float measured_distance, float real_width, float width_in_rf_image)
{
    // Finding the focal length
    return (width_in_rf_image * measured_distance) / real_width;
}

// Distance estimation function
float Distance_finder(float Focal_Length, float real_face_width, float face_width_in_frame)
{
    // Calculate the distance
    return (real_face_width * Focal_Length) / face_width_in_frame;
}

// Function to detect face width in an image
float face_data(Mat &image)
{
    float face_width = 0; // Set face width to zero initially

    // Convert color image to grayscale image
    Mat gray_image;
    cvtColor(image, gray_image, COLOR_BGR2GRAY);

    // Detect faces in the image
    vector<Rect> faces;
    face_detector.detectMultiScale(gray_image, faces, 1.3, 5);

    // Loop through the detected faces and get coordinates and dimensions
    for (size_t i = 0; i < faces.size(); ++i)
    {
        // Draw a rectangle around the face
        rectangle(image, faces[i], GREEN, 2);

        // Get face width in pixels
        face_width = faces[i].width;
    }

    // Return the face width in pixels
    return face_width;
}

int main()
{
    // Load the pre-trained Haar Cascade for face detection
    if (!face_detector.load("src/robocon_detector/model/haarcascade_frontalface_default.xml"))
    {
        cerr << "Error loading face detection model!" << endl;
        return -1;
    }

    // Read the reference image from directory
    Mat ref_image = imread("src/robocon_detector/assets/Ref_image.png");

    if (ref_image.empty())
    {
        cerr << "Reference image not found!" << endl;
        return -1;
    }

    // Find the face width in the reference image
    float ref_image_face_width = face_data(ref_image);

    // Calculate the focal length by calling "Focal_Length_Finder"
    float Focal_length_found = Focal_Length_Finder(Known_distance, Known_width, ref_image_face_width);

    // Print the found focal length
    cout << "Focal Length: " << Focal_length_found << endl;

    // Show the reference image
    imshow("Reference Image", ref_image);
    waitKey(0); // Wait for a key press before closing the reference image window

    // Initialize the camera object to get frames from it
    VideoCapture cap("src/robocon_detector/assets/Ref_Video.mp4");
    if (!cap.isOpened())
    {
        cerr << "Error opening video stream!" << endl;
        return -1;
    }

    // Loop through the frames from the camera
    while (true)
    {
        Mat frame;
        cap >> frame; // Capture a frame from the camera

        if (frame.empty())
        {
            break;
        }

        // Call the face_data function to find the face width in the frame
        float face_width_in_frame = face_data(frame);

        // Check if the face is detected (face_width_in_frame != 0)
        if (face_width_in_frame != 0)
        {
            // Calculate the distance by calling the Distance_finder function
            float distance = Distance_finder(Focal_length_found, Known_width, face_width_in_frame);

            // Draw a line as background for the text
            // line(frame, Point(30, 30), Point(230, 30), RED, 32);
            // line(frame, Point(30, 30), Point(230, 30), BLACK, 28);

            // Draw text on the frame with the calculated distance
            putText(frame, "Distance: " + to_string(round(distance)) + " CM", Point(300, 35), fonts, 0.6, GREEN, 2);
        }

        // Show the frame on the screen
        imshow("Frame", frame);

        // Quit the program if 'q' is pressed
        if (waitKey(1) == 'q')
        {
            break;
        }
    }

    // Release the camera
    cap.release();

    // Close all windows
    destroyAllWindows();

    return 0;
}
