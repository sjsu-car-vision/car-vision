#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <stdio.h>
#include <string>

using namespace std;
using namespace cv;


/*
 * Detecting people in frame using HOG
 * Using HOG's pre-trained model
 */

int main(int argc, const char * argv[])
{
	/* Using primary camera; limit frame to increase performance */
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	if (!cap.isOpened())
		return -1;

	/* Initialize frame, window, and HOG object */
	Mat img;
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	namedWindow("video capture", CV_WINDOW_AUTOSIZE);

	/* LOOP */
	while (true)
	{
		/* Get frame from video source */
		cap >> img;
		if (!img.data)
			continue;

		/* Initialize dynamic array for detected objects */
		vector<Rect> found;
		vector<Rect> found_filtered;

		/* Process frame and detect people
		 * Store found objects as Rectangle objects
		 */
		hog.detectMultiScale(img, found, 0, Size(8, 8), Size(32, 32), 1.05, 1);

		/* Enter FOR loop if any objects found */
		size_t i, j;
		for (i = 0; i<found.size(); i++)
		{
			/* Print number of found objects and its coordinates */
			printf("Found:%d\t", found.size());
			printf("X:%d, Y:%d\n", found[i].x, found[i].y);
			fflush(stdout);


			Rect r = found[i];
			for (j = 0; j<found.size(); j++)
				if (j != i && (r & found[j]) == r)
					break;
			if (j == found.size())
				found_filtered.push_back(r);
		}
		for (i = 0; i<found_filtered.size(); i++)
		{
			Rect r = found_filtered[i];
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.06);
			r.height = cvRound(r.height*0.9);
			rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
		}


		imshow("video capture", img);
		if (waitKey(20) >= 0)
			break;
	}

	return 0;
}
