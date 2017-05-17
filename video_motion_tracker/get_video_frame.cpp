/*
* Copyright (c) 2016 Rockchip Electronics Co. Ltd.
* Get all of the imge frame from a video file
* Author: SYJ
*/
#include "get_video_frame.h"
using namespace cv;

bool get_video_frame(char* video_address, char* frame_address)
{
	std::cout << "video_address:" << video_address<<std::endl;
	std::cout << "frame_address:" << frame_address << std::endl;
#if 1
	CvCapture *capture;
	capture = cvCreateFileCapture(video_address);
	assert(capture != NULL);

	int frameH = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	int frameW = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	int fps = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	int numFrames = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
	printf("\tvideo height : %d\n\tvideo width : %d\n\tfps : %d\n\tframe numbers : %d\n", frameH, frameW, fps, numFrames);

	IplImage *img_frame = 0;
	int frame_num = 0;
	char image_name[256];

	while (img_frame = cvQueryFrame(capture))
	{
		std::cout << "\rExtract frame£º" << frame_num ;
		fflush(stdout);
		sprintf(image_name, "%s%s%d%s", frame_address, "\\frame", ++frame_num, ".jpg");
		cvSaveImage(image_name, img_frame);
		if (frame_num == numFrames)
			break;
	}
	cvReleaseCapture(&capture);
#endif
	return 0;
}

