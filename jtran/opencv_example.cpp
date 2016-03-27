#include <iostream>
#include <stdio.h>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

int display_image(std::string file_path);
int display_video_cam(int cam_id);
int display_image_cam_filtered(int cam_id, bool trace=false);

using namespace cv;


int main(void) {
	int error = 0;
	//error = display_image("C:\\Users\\Jason\\Desktop\\cv\\Capture.JPG");
	//error = display_video_cam(0);
	error = display_image_cam_filtered(0, true);

	return error;
}


int display_image(std::string file_path) {
	int error = 0;
	Mat img = imread(file_path.c_str(), CV_LOAD_IMAGE_UNCHANGED);
	if (img.empty()) {
		std::cout << "ERROR: Image cannot be loaded!" << std::endl;
		error = -1;
	}
	else {
		namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
		imshow("MyWindow", img);

		waitKey(0);

		destroyWindow("MyWindow");
	}
	return error;
}


int display_video_cam(int cam_id) {
	int error = 0;
	VideoCapture cap(cam_id);

	if (!cap.isOpened()) {
		printf("ERROR: failed to initialize camera: %d\n", cam_id);
		error = 1;
		return error;
	}

	double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	printf("Frame size: %.0fx%.0f\n", width, height);

	while (1) {
		Mat frame;
		bool success = cap.read(frame);

		if (!success) {
			break;
		}

		imshow("MyVideo", frame);

		if (waitKey(30) == 27) {
			printf("esc detected!\n");
			break;
		}
	}

	return error;
}


int display_image_cam_filtered(int cam_id, bool trace) {
	int error = 0;
	VideoCapture cap(cam_id);
	if (!cap.isOpened()) {
		printf("ERROR: failed to initialize camera: %d", cam_id);
		error = 1;
		return error;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE);

	int iLowH = 170;
	int iHighH = 179;

	int iLowS = 160;
	int iHighS = 255;

	int iLowV = 60;
	int iHighV = 255;

	cvCreateTrackbar("LowH", "Control", &iLowH, 179);
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255);
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255);
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	/* Object tracking feature */
	int iLastX = -1;
	int iLastY = -1;
	Mat imgTmp;
	cap.read(imgTmp);
	Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);

	while (1) {
		Mat imgOriginal;
		bool success = cap.read(imgOriginal);
		if (!success) {
			break;
		}

		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

		Mat imgThresholded;

		/* FILTERING FUNCTION */
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		/* Object tracking feature */
		Moments oMoments = moments(imgThresholded);
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		if (dArea > 10000) {
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;

			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0) {
				line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0, 0, 255), 2);
			}

			iLastX = posX;
			iLastY = posY;
		}

		if (trace){
			imgOriginal = imgOriginal + imgLines;
		}

		imshow("Thresholded Image", imgThresholded);
		imshow("Original", imgOriginal);

		if (waitKey(30) == 27) {
			printf("esc detected!\n");
			break;
		}
	}

	return error;
}