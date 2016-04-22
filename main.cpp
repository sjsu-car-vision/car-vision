#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    /* Opens Camera feed */
    VideoCapture cap("/Users/jianganson72/Desktop/Test/IMG_0462.MOV");
    if(!cap.isOpened()){
        cout << "Camera failed to open" << endl;
        return -1;
    }
    
    /* Create a window called "Control"
     * Will be used to change HSV values
     */
    namedWindow("Control", CV_WINDOW_AUTOSIZE);
    
    /* HSV Values on what is considered Red */
    int iLowH = 170;
    int iHighH = 179;
    
    int iLowS = 200;
    int iHighS = 255;
    
    int iLowV = 60;
    int iHighV = 255;
    
    /* Create trackbars in "Control" window
     * Move trackbar to make necessary adjustments to limit false positives
     */
    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);
    
    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);
    
    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    
    while(true) {
        Mat frame;
        Mat HSV;
        Mat Threshold;
        Mat Threshold_output;
        
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        
        cap.read(frame);
        
        /* Convert frame from BGR to HSV */
        cvtColor(frame, HSV, COLOR_BGR2HSV);
        
        /* Checks if object is red */
        inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold);
        inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold_output);
        
        /* Filtering images */
        erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        
        dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        
        /* Separates red objects, in this case, traffic lights */
        threshold(Threshold, Threshold, 100, 255, THRESH_BINARY_INV);
        
        /* Finds contours of red objects */
        findContours(Threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        
        /* Declaring contour variables */
        vector<vector<Point>> contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f> center(contours.size());
        vector<float> radius(contours.size());
        
        /* Calculates upright bounding rectangle of a point set */
        for(int i = 0; i < contours.size(); i++){
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(Mat(contours_poly[i]));
        }
        
        Mat drawing = Mat::zeros(Threshold_output.size(), CV_8UC3);
        
        /* Draws a rectangle around the traffic light object */
        for(int i = 0; i < contours.size(); i++){
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 100, 255));
        }
        
        /* Shows Threshold Feed */
        imshow("Threshold Image", Threshold_output);
        
        frame = frame + drawing; // This makes the video feed include the contour drawings
        namedWindow("Camera_feed", CV_WINDOW_AUTOSIZE);
        imshow("Camera_feed", frame);
        
        if (waitKey(1) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }
    
    return 0;
}
