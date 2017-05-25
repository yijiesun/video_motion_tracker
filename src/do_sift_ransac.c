/*
* Copyright (c) 2017 Rockchip Electronics Co. Ltd.
* Do sift&ransac algo for each pair of frame
* Author: SYJ
*/

#include "do_sift_ransac.h"

void do_sift_ransac_algo(char* frame_address, char* result_address, int frame_cnt, int direct)
{
	char* frame_name_1[256];
	char* frame_name_2[256];
	char* save_name[256];

	for (int i = 1; i <= frame_cnt; i++) {
		printf("\nframe:%d\n", i);
		sprintf(frame_name_1, "%s%s%d%s", frame_address, "\\frame", i, ".jpg");
		sprintf(frame_name_2, "%s%s%d%s", frame_address, "\\frame", i + 1, ".jpg");

		IplImage* img1, *img2;
		img1 = cvLoadImage(frame_name_1, 1);
		img2 = cvLoadImage(frame_name_2, 1);

		if (img1 == NULL || img2 == NULL)
		{
			printf("file not exist or end of sequence");
			return;
		}
#if TEN_PIECE
		for (float j = 0.0; j < 1.0; j += 0.1)
		{
			sprintf(save_name, "%s%s%.1f%s", frame_address, "\\frame", i + j, ".jpg");
			IplImage* img1_block, *img2_block;
			cvSetImageROI(img1, cvRect(0, j*img1->height, img1->width, 0.1*img1->height));
			img1_block = cvCreateImage(cvSize(img1->width, 0.1*img1->height), IPL_DEPTH_8U, img1->nChannels);
			cvCopy(img1, img1_block, 0);
			cvResetImageROI(img1);

			cvSetImageROI(img2, cvRect(0, j*img2->height, img2->width, 0.1*img2->height));
			img2_block = cvCreateImage(cvSize(img2->width, 0.1*img2->height), IPL_DEPTH_8U, img2->nChannels);
			cvCopy(img2, img2_block, 0);
			cvResetImageROI(img2);

			match(img1_block, img2_block, save_name, result_address, i + j, direct);
			cvReleaseImage(&img1_block);
			cvReleaseImage(&img2_block);
		}
#else
		match(img1, img2, save_name, result_address, i, direct);
#endif
		cvReleaseImage(&img1);
		cvReleaseImage(&img2);
	}
}

void match(IplImage *img1, IplImage *img2, char* savename, char* txtFile, float t_time, int direct)
{
	/* the maximum number of keypoint NN candidates to check during BBF search */
	int KDTREE_BBF_MAX_NN_CHKS = 200;

	/* threshold on squared ratio of distances between NN and 2nd NN */
	float NN_SQ_DIST_RATIO_THR = 0.49;

	struct feature *feat1, *feat2;//feat1：图1的特征点数组，feat2：图2的特征点数组  
	int n1 = 0, n2 = 0;//n1:图1中的特征点个数，n2：图2中的特征点个数  
	struct feature *feat;//每个特征点  
	struct kd_node *kd_root;//k-d树的树根  
	struct feature **nbrs;//当前特征点的最近邻点数组  
	int matchNum = 0;//经距离比值法筛选后的匹配点对的个数  
	struct feature **inliers;//精RANSAC筛选后的内点数组  
	int n_inliers = 0;//经RANSAC算法筛选后的内点个数,即feat1中具有符合要求的特征点的个数  

	n1 = sift_features(img1, &feat1);//检测图1中的SIFT特征点,n1是图1的特征点个数  

	n2 = sift_features(img2, &feat2);//检测图2中的SIFT特征点，n2是图2的特征点个数  

	double d0, d1;//feat1中每个特征点到最近邻和次近邻的距离  
	matchNum = 0;//经距离比值法筛选后的匹配点对的个数 

#if SAVE_SIFT_IMAGE
	CvPoint pt1, pt2;//连线的两个端点  
	IplImage  *stacked_ransac;
	stacked_ransac = stack_imgs(img1, img2);//合成图像，显示经RANSAC算法筛选后的匹配结果  
#endif											
	kd_root = kdtree_build(feat2, n2);//根据图2的特征点集feat2建立k-d树，返回k-d树根给kd_root  
	int n = min(n1, n2);

	//遍历特征点集feat1，针对feat1中每个特征点feat，选取符合距离比值条件的匹配点，放到feat的fwd_match域中  
	for (int i = 0; i < n; i++)
	{
		feat = feat1 + i;//第i个特征点的指针  
						 //在kd_root中搜索目标点feat的2个最近邻点，存放在nbrs中，返回实际找到的近邻点个数  
		int k = kdtree_bbf_knn(kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS);
		if (k == 2)
		{
			d0 = descr_dist_sq(feat, nbrs[0]);//feat与最近邻点的距离的平方  
			d1 = descr_dist_sq(feat, nbrs[1]);//feat与次近邻点的距离的平方  
											  //若d0和d1的比值小于阈值NN_SQ_DIST_RATIO_THR，则接受此匹配，否则剔除  

			if (d0 < d1 * NN_SQ_DIST_RATIO_THR)
			{   //将目标点feat和最近邻点作为匹配点对  
#if DRAW_RESULT_NO_RANSAC
				pt1 = cvPoint(cvRound(feat->x), cvRound(feat->y));
				pt2 = cvPoint(cvRound(nbrs[0]->x), cvRound(nbrs[0]->y));
				pt2.y += img1->height;//由于两幅图是上下排列的，pt2的纵坐标加上图1的高度，作为连线的终点  
				cvLine(stacked_ransac, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//画出连线  
#endif
				matchNum++;//统计匹配点对的个数  
				feat1[i].fwd_match = nbrs[0];//使点feat的fwd_match域指向其对应的匹配点  
			}
		}
		if (nbrs)
			free(nbrs);//释放近邻数组  
	}

	const char *IMG_MATCH2 = "stacked_ransac";
	printf("SIFT匹配点对个数：%d\n", matchNum);

	//利用RANSAC算法筛选匹配点,计算变换矩阵H  
	CvMat * H = ransac_xform(feat1, n, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, homog_xfer_err, 3.0, &inliers, &n_inliers);

	printf("经RANSAC去噪后匹配点对个数：%d\n", n_inliers);

	double transation = 0.0;
	int count = 0;
	//遍历经RANSAC算法筛选后的特征点集合inliers，找到每个特征点的匹配点，画出连线  
	for (int i = 0; i < n_inliers; i++)
	{
		feat = inliers[i];//第i个特征点  
#if SET_ROI
		if (cvRound(feat->y) < 480 || cvRound(feat->y) > 500 && cvRound(feat->y) < 980) {//避免图像中不做匹配的区域
#endif
			if (direct == 0)
				transation = transation + feat->x - feat->fwd_match->x;
			else
				transation = transation + feat->y - feat->fwd_match->y;
#if SAVE_SIFT_IMAGE
			pt1 = cvPoint(cvRound(feat->x), cvRound(feat->y));//图1中点的坐标  
			pt2 = cvPoint(cvRound(feat->fwd_match->x), cvRound(feat->fwd_match->y));//图2中点的坐标(feat的匹配点)  
			pt2.y += img1->height;//由于两幅图是上下排列的，pt2的纵坐标加上图1的高度，作为连线的终点  
			cvLine(stacked_ransac, pt1, pt2, CV_RGB(0, 255, 0), 1, 8, 0);//画出连线  
#endif
			count++;
#if SET_ROI
		}
#endif
	}
	if (count == 0)
		count = 1;

	double tranX = transation / count*1.0;
	printf("位移量:%lf\n", tranX);

	if (tranX != 0)
	{
		FILE* file;
		file = fopen(txtFile, "a+");
		fprintf(file, "%.1f%s", t_time, ",");
		fprintf(file, "%lf\n", tranX);
		fclose(file);
	}
#if SAVE_SIFT_IMAGE
	cvSaveImage(savename, stacked_ransac, 0);
	cvReleaseImage(&stacked_ransac);
#endif
	if (n1)
		free(feat1);
	if (n2)
		free(feat2);
	if (n_inliers)
		free(inliers);
	if (n2)
		free(kd_root);
}