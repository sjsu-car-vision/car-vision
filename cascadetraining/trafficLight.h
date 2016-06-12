//
//  trafficLight.h
//  HaarRedLightDetector
//
//  Created by Anson Jiang on 6/11/16.
//  Copyright © 2016 Anson Jiang. All rights reserved.
//

#ifndef trafficLight_h
#define trafficLight_h

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"

using namespace cv;

class trafficLight {
public:
    void detectAndDisplay(Mat image, CascadeClassifier cascadeFile);
    void colorDetect(Mat frame);
};


#endif /* trafficLight_h */
