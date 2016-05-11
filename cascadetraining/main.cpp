//
//  main.cpp
//  HaarRedLightDetector
//
//  Created by Anson Jiang on 5/10/16.
//  Copyright © 2016 Anson Jiang. All rights reserved.
//

#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);

string cascade = "/Users/jianganson72/Desktop/data/cascade.xml";
CascadeClassifier trafficLightCascade;

int main(int argc, const char * argv[])
{
    // insert code here...
    Mat image;
    
    if(!trafficLightCascade.load(cascade))
    {
        cout << "Cannot load cascade" << endl;
        return -1;
    }
    
    image = imread("/Users/jianganson72/Desktop/test1.jpg", CV_LOAD_IMAGE_UNCHANGED);

    if(image.empty()){
        cout << "cannot read image" << endl;
        return -1;
    }
    
    detectAndDisplay(image);
    
    return 0;
}

void detectAndDisplay(Mat image)
{
    std::vector<Rect> trafficlights;
    Mat frame_gray;
    cvtColor(image, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    
    trafficLightCascade.detectMultiScale(frame_gray, trafficlights, 1.1, 3, 0|CASCADE_SCALE_IMAGE, Size(50, 50));
    
    for(size_t i = 0; i < trafficlights.size(); i++)
    {
        rectangle(image, cvPoint(trafficlights[i].x - trafficlights[i].width/2, trafficlights[i].y - trafficlights[i].height/2),
                         cvPoint(trafficlights[i].x + trafficlights[i].width/2, trafficlights[i].y + trafficlights[i].height/2/2), Scalar(0, 100, 255));
    }
    
    namedWindow("Display Window", WINDOW_AUTOSIZE);
    imshow("Display Window", image);
    
    waitKey(0);

}
