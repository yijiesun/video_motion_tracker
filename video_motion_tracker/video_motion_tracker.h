#pragma once
#include "include.h"

IplImage* img_cur, *img_ref;

char* video_address;
char result_address[_MAX_DIR];
char frame_address[_MAX_DIR];
char drive[_MAX_DRIVE];
char dirct[_MAX_DIR];
char fname[_MAX_FNAME];
char ext[_MAX_EXT];

void before_algo(char* argv[]);