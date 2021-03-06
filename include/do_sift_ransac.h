#pragma once
#ifndef DO_SIFT_RANSAC_H
#define DO_SIFT_RANSAC_H

#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <stdio.h>
#include<math.h>  

#define SAVE_SIFT_IMAGE 0 //保存SIFT匹配点连线图
#define DRAW_RESULT_NO_RANSAC 0 //保存未经过RANSAC去噪的连线图
#define SET_ROI 0 //设置不检查SIFT匹配点的区域
#define TEN_PIECE 1 //是否分10块检测SIFT
#ifdef __cplusplus
extern "C" {
#endif

	void match(IplImage *img1, IplImage *img2, char* savename, char* txtFile, float t_time, int direct);
	void do_sift_ransac_algo(char* frame_address, char* result_address, int frame_cnt, int direct);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !DO_SIFT_RANSAC_H