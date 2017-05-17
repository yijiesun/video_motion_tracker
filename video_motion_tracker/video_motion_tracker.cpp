/*
* Copyright (c) 2016 Rockchip Electronics Co. Ltd.
* Do video motion tracker with SIFT&RANSAC algo
* Author: SYJ
*/
#include "video_motion_tracker.h"
#include "get_video_frame.h"
#include "do_sift_ransac.h"

int main(int argc, char* argv[])
{
	before_algo(argv);
	int frame_num=get_video_frame(video_address, frame_address);
	bool ret_sift = do_sift_ransac(frame_address, result_address);
	getchar();
    return 0;
}

void before_algo(char* argv[])
{
	video_address = argv[1];
	_splitpath(video_address, drive, dirct, fname, ext);

	sprintf(result_address, "%s%s%s", drive, dirct,"video_sift.txt");
	sprintf(frame_address, "%s%s%s", drive, dirct, "frame");
	char mkdir[256];
	sprintf(mkdir, "%s%s", "md ", frame_address);
	system(mkdir);
}
