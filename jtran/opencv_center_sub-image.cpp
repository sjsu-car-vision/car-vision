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


/* Print mouse coordinates to console */
void CallBackFunc(int event, int x, int y, int flags, void* userdata);


int main(int argc, const char * argv[])
{
	int error = 0;

	/* Create windows */
	namedWindow("VIDEOFEED", CV_WINDOW_AUTOSIZE);
	namedWindow("TEMPLATE", CV_WINDOW_AUTOSIZE);

	/* Configure mouse tracking in the window VIDEOFEED */
	setMouseCallback("VIDEOFEED", CallBackFunc, NULL);

	/* Get video source */
	int cam_id = 0;
	VideoCapture cap(cam_id);
	if (!cap.isOpened()) {
		printf("ERROR: failed to initialize camera: %d\n", cam_id);
		error = 1;
		return error;
	}

	/* Continuously analyze video source */
	while(1) {
		Mat frame;
		bool success = cap.read(frame);
		if (!success) {
			printf("ERROR: failed to get frame from source\n");
			break;
		}

		/*
		 *
		 * Image processing implementation goes HERE
		 *
		 */

		/* Get template from center of frame
		 *	- Get center coordinates
		 *	- Create template at the center
		 */
		const double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		const double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

		const double x_center = width/2;
		const double y_center = height/2;

		const double box_length = x_center/2;
		const double box_height = y_center/2;

		Rect center_box(x_center - box_length/2,
						y_center - box_height/2,
						box_length,
						box_height);

		/* Assign template as a sub-image of frame
		 * Cropping center
		 */
		Mat img_template(frame, center_box);

		/* Display both full image and sub-image */
		imshow("VIDEOFEED", frame);
		imshow("TEMPLATE", img_template);

		if (waitKey(30) == 27) {
			printf("Terminating...\n");
			break;
		}
	} // END while

	return 0;
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}
