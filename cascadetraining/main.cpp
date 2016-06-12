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
#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"
#include "trafficLight.h"

using namespace std;
using namespace cv;

string cascade = "/Users/jianganson72/Desktop/trafficLightTraining/data/cascade.xml";
CascadeClassifier trafficLightCascade;

int main(int argc, const char * argv[])
{
    // insert code here...
    trafficLight t;
    
    Mat image;
    
    VideoCapture cap("/Users/jianganson72/Desktop/My Movie.mp4");
    
    if(!trafficLightCascade.load(cascade))
    {
        cout << "Cannot load cascade" << endl;
        return -1;
    }
    
    
    /* For videos */
    //cap.open( -1 );
    if ( ! cap.isOpened() ) { printf("--(!)Error opening video capture\n"); return -1; }

    while ( cap.read(image) )
    {
        if( image.empty() )
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }
        
        //-- 3. Apply the classifier to the frame
        t.colorDetect(image);
        t.detectAndDisplay(image, trafficLightCascade);
        
        int c = waitKey(10);
        if( (char)c == 27 ) { break; } // escape
    }
    
    /* For images */
    /*
     
    image = imread("/Users/jianganson72/Desktop/20141115_154224.jpg", CV_LOAD_IMAGE_UNCHANGED);
    image = imread("/Users/jianganson72/Desktop/Test/test1.jpg", CV_LOAD_IMAGE_UNCHANGED);

    if(image.empty()){
        cout << "cannot read image" << endl;
        return -1;
    }
    
    t.detectAndDisplay(image);
     
    int c = waitKey(10);
    if( (char)c == 27 ) { break; } // escape

    */
    
    return 0;
    
}