#pragma once
#ifndef VIDEO_MOTION_TRACKER_H
#define VIDEO_MOTION_TRACKER_H

#include "include.h"

IplImage* img_cur, *img_ref;

char* video_address;
char video_direct[_MAX_DIR];
char result_address[_MAX_DIR];
char frame_address[_MAX_DIR];
char drive[_MAX_DRIVE];
char dirct[_MAX_DIR];
char fname[_MAX_FNAME];
char ext[_MAX_EXT];

int sift_direct;
enum direct
{
	horizion,
	verticl
};

void before_algo(char* argv[]);

#endif // !VIDEO_MOTION_TRACKER_H