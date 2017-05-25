#pragma once
#ifndef GET_VIDEO_FRAME_H
#define GET_VIDEO_FRAME_H

#include "include.h"
#define EXTRACT_WATERMARKS 0 //是否提取水印数据
int get_video_frame(char* video_address, char* frame_address, char* video_direct, char* fname);
void extract_watermarks(IplImage* src, char* data_file, int frame_num, int frame_cnt);
int dataDenoise(char *file, char*fileo);
#endif  // !GET_VIDEO_FRAME_H