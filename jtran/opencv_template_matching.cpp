/* BASED OFF OF opencv_center_sub-image.cpp */
/* PRESS c TO CAPTURE A TEMPLATE */

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


/*
 * Press c to capture a template
 */

int main(int argc, const char * argv[])
{
	/* Init variables */
	int error = 0;
	Mat img_template;
	bool template_obtained = false;

	/* Create windows */
	namedWindow("VIDEOFEED", CV_WINDOW_AUTOSIZE);
	namedWindow("TEMPLATE", CV_WINDOW_AUTOSIZE);
	//namedWindow("RESULT", CV_WINDOW_AUTOSIZE);

	/* Configure mouse tracking in the window VIDEOFEED */
	//setMouseCallback("VIDEOFEED", CallBackFunc, NULL);

	/* Get video source */
	int cam_id = 0;
	VideoCapture cap(cam_id);
	if (!cap.isOpened()) {
		printf("ERROR: failed to initialize camera: %d\n", cam_id);
		error = 1;
		return error;
	}


	/* Get a box at the center of the video source */
	const double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	const double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	const double x_center = width/2;
	const double y_center = height/2;

	/* Setting box size to be 1/4 of the video source */
	const double box_length = x_center/2;
	const double box_height = y_center/2;

	Rect center_box(x_center - box_length/2,
					y_center - box_height/2,
					box_length,
					box_height);

	/* Continuously analyze video source */
	while(1) {
		Mat frame;
		bool success = cap.read(frame);
		if (!success) {
			printf("ERROR: failed to get frame from source\n");
			break;
		}

		/*
		 * --- Image processing implementation goes HERE ---
		 */

		if (template_obtained) {
			Mat result;
			int result_cols = frame.cols - img_template.cols + 1;
			int result_rows = frame.rows - img_template.rows + 1;
			result.create(result_rows, result_cols, CV_32FC1);

			/* Perform template matching, then normalize */
			enum TemplateMatchModes match_method = TM_SQDIFF;
			matchTemplate(frame, img_template, result, match_method);
			normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

			/* Get match location */
			double min_val, max_val;
			Point min_loc, max_loc;
			Point match_loc;
			minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, Mat());
			if (match_method == TM_SQDIFF || match_method == TM_SQDIFF_NORMED) {
				match_loc = min_loc;
			} else {
				match_loc = max_loc;
			}

			rectangle(frame,
					  match_loc,
					  Point(match_loc.x + img_template.cols, match_loc.y + img_template.rows),
					  Scalar::all(0), 2, 8, 0);

//			rectangle(result,
//					  match_loc,
//					  Point(match_loc.x + img_template.cols, match_loc.y + img_template.rows),
//					  Scalar::all(0), 2, 8, 0);

			//imshow("RESULT", result);

			printf("X:%d, Y:%d\n", match_loc.x, match_loc.y);
			fflush(stdout);
		}

		/* Display both full image and sub-image */
		imshow("VIDEOFEED", frame);

		/* Check key event */
		int ascii_dec = waitKey(30);
		if (ascii_dec == 99) {
			/* Get template from center of frame
			 * - Assign template as a sub-image of frame
			 * - Cropping center
			 */
			Mat temp(frame, center_box);
			img_template = temp;
			imshow("TEMPLATE", img_template);
			template_obtained = true;
		}
		else if (ascii_dec == 27) {
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
