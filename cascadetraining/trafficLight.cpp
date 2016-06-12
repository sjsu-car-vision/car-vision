//
//  trafficLight.cpp
//  HaarRedLightDetector
//
//  Created by Anson Jiang on 6/11/16.
//  Copyright © 2016 Anson Jiang. All rights reserved.
//

#include <stdio.h>
#include "trafficLight.h"

using namespace std;
using namespace cv;

void trafficLight::detectAndDisplay(Mat image, CascadeClassifier cascadeFile)
{
    std::vector<Rect> trafficlights;
    Mat frame_gray;
    
    Rect roi(350, 10, 150, 250);
    
    Mat imageROI = image(roi);
    
    rectangle(image, roi, Scalar(0, 0, 255), 2);
    
    cvtColor(imageROI, frame_gray, CV_BGR2GRAY);
    
    equalizeHist(frame_gray, frame_gray);
    
    cascadeFile.detectMultiScale(frame_gray, trafficlights, 1.1, 3, 0|CASCADE_SCALE_IMAGE, Size(24, 48));
    
    for(size_t i = 0; i < trafficlights.size(); i++)
    {
        rectangle(imageROI, trafficlights[i].tl(), trafficlights[i].br(), Scalar(0, 0, 255), 2);
    }
    
    namedWindow("Display Window", WINDOW_AUTOSIZE);
    imshow("Display Window", imageROI);
    
    namedWindow("Window", WINDOW_AUTOSIZE);
    imshow("Window", image);
}

void trafficLight::colorDetect(Mat frame)
{
    Mat HSV;
    Mat Threshold;
    Mat Threshold_output;
    
    
    /* HSV Values on what is considered Red */
    int iLowH = 170;
    int iHighH = 179;
    
    int iLowS = 200;
    int iHighS = 255;
    
    int iLowV = 60;
    int iHighV = 255;
    
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    
    cvtColor(frame, HSV, CV_BGR2HSV);
    
    inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold);
    inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold_output);
    
    erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    
    dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    
    threshold(Threshold, Threshold, 100, 255, THRESH_BINARY_INV);
    
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
    
    imshow("Threshold Window", Threshold_output);
}

