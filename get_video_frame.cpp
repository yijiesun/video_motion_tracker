/*
* Copyright (c) 2017 Rockchip Electronics Co. Ltd.
* Get all of the imge frame from a video file
* Author: SYJ
*/

#include "get_video_frame.h"
using namespace cv;

int dist[32] = {
4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
3,
4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
3,
4, 4, 4
};
int colDist[3] = { 5,4,4 };
int rowDist[9] = { 21,21,20,21,21,20,21,21,21 };

int get_video_frame(char* video_address, char* frame_address,char* video_direct,char* fname)
{
	std::cout << "video_address:" << video_address<<std::endl;
	std::cout << "frame_address:" << frame_address << std::endl;

	CvCapture *capture;
	capture = cvCreateFileCapture(video_address);
	assert(capture != NULL);

	int frameH = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	int frameW = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	int fps = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	int numFrames = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
	printf("\tvideo height : %d\n\tvideo width : %d\n\tfps : %d\n\tframe numbers : %d\n", frameH, frameW, fps, numFrames);

	char data_file[256];
	char de_data_file[256];
	char command_file[256];
	sprintf(data_file, "%s%s", video_direct, "gyro_total.txt");
	sprintf(de_data_file, "%s%s", video_direct, "gyro.txt");
	sprintf(command_file, "%s%s", video_direct, "command.tmp");
	IplImage *img_frame = 0;
	int frame_num = 0;
	char image_name[256];

	while (img_frame = cvQueryFrame(capture))
	{
		std::cout << "\rExtract frame：" << frame_num ;
		fflush(stdout);
		sprintf(image_name, "%s%s%d%s", frame_address, "\\frame", ++frame_num, ".jpg");
		cvSaveImage(image_name, img_frame);

		extract_watermarks(img_frame, data_file, command_file, frame_num, numFrames);

		if (frame_num == numFrames)
			break;
	}
	cvReleaseCapture(&capture);

	int gyroCount = 0;
	gyroCount = dataDenoise(data_file, de_data_file);

	return frame_num;
}

void extract_watermarks(IplImage* src, char* data_file, char* command_file, int frame_num, int frame_cnt)
{
	IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);//创建目标图像 
	cvCvtColor(src, dst, CV_BGR2GRAY);

	bool goodData = false;

	int width = dst->width;//图片宽度
	int height = dst->height;//图片高度

	std::string binaryData = "";
	int gyroTmp[4][7] = { 0 };
	int gyro[7][4] = { 0 };

	int gbegin = 0;
	int gend = 0;
	int data[32] = { 0 };
	int datacommand[32] = { 0 };
	int intensity = 0;

	CvPoint   centerpoint;
	centerpoint.x = 169 + 1;
	centerpoint.y = 987 + 1;

	for (int l = 0; l < 4; l++) {
		uchar* ptr = (uchar*)dst->imageData + (centerpoint.y)*dst->width;//获得灰度值数据指针

		for (int j = 0; j < 7; j++)
		{
			for (int i = 0; i < 32; i++)
			{
				intensity = ptr[centerpoint.x];
				data[i] = intensity;
				if (intensity > 200)
				{
					data[i] = 1;
				}
				else if (intensity < 55)
				{
					data[i] = 0;
				}

				else {
					data[i] = 2;
				}

				//cvRectangle(dst, centerpoint, cvPoint(centerpoint.x, centerpoint.y), cvScalar(125, 125, 125),1, 8, 0);
				centerpoint.x = centerpoint.x + dist[i];
			}

			/* remove not interest num */
			for (int y = 30; y >= 0; y--)
			{
				if (data[y] == 0 && data[y - 1] == 1 && data[y - 2] == 0)//黑白黑
				{
					data[y] = 2;
					data[y - 1] = 2;
					data[y - 2] = 2;
					break;
				}
				if (data[y] != 0 || data[y - 1] != 1 || data[y - 2] != 0)
				{
					data[y] = 2;
				}
			}

			for (int x = 0; x < 32; x++)
			{
				if (data[x] != 2)
					binaryData = binaryData + std::to_string(data[x]);
				else
					break;
			}
			/* 二进制字符串转整数 */
			char* ss = (char*)binaryData.data();
			char* e;
			int out = strtol(ss, &e, 2);
			if (data[0] == 0) //正负判断
				out = -out;

			gyroTmp[l][j] = out;
			binaryData = "";
			centerpoint.x = centerpoint.x + rowDist[j];
		}
		centerpoint.y = centerpoint.y + 20 + colDist[l];
		centerpoint.x = 169 + 1;
	}

	/* 转置 */
	for (int m = 0; m < 4; m++)
	{
		for (int n = 0; n < 7; n++)
		{
			gyro[n][m] = gyroTmp[m][n];
		}
	}
	FILE* of_data, *of_command;
	of_data = fopen(data_file, "a+");
	of_command = fopen(command_file, "a+");

	for (int m = 0; m < 6; m++)
	{
		for (int n = 0; n < 4; n++)
		{
			if (gyro[m][0] != 0 && gyro[m][1] != 0 && gyro[m][2] != 0 && gyro[m][3] != 0) {
				fprintf(of_data, "%d%s", gyro[m][n], ",");
			goodData = true;
		}
			else goodData = false;
	}
		if (goodData)
			fprintf(of_data, "%s","\n");
	}
	if (frame_num == 1) //保存第一帧曝光时间戳
		fprintf(of_command, "%d", gyro[6][1]);
	if (frame_num == frame_cnt)//保存最后一帧曝光时间戳
		fprintf(of_command, "%s%d%s%d%s", " ",gyro[6][1]," ", frame_num - 1," ");

	fclose(of_data);
	fclose(of_command);

}

int dataDenoise(char *file, char*fileo)
{
	using namespace std;
	ifstream in(file);
	ofstream outfile(fileo, ios::out | ios::trunc);
	string filename;
	string line1;
	string line2;
	string tmp;
	int a1, a3, a4;
	int count = 1;
	getline(in, line1);
	outfile << line1 << endl;
	tmp = line1;
	char *str1 = (char *)tmp.c_str();//string --> char
	const char *split = ",";
	char *p1 = strtok(str1, split);//逗号分隔依次取出
	sscanf(p1, "%d", &a1);//char ---> int

	char *d3 = strtok(NULL, split);//取出陀螺仪X值
	sscanf(d3, "%d", &a3);//char ---> int

	while (getline(in, line2)) {
		tmp = line2;
		char *str2 = (char *)tmp.c_str();//string --> char
		char *p2 = strtok(str2, split);//逗号分隔依次取出
		int a2;
		sscanf(p2, "%d", &a2);//char ---> int	

		char *d4 = strtok(NULL, split);//逗号分隔依次取出
		sscanf(d4, "%d", &a4);//char ---> int

		if (a2 > a1&&a2 - a1<100000 && abs(a4 - a3)<10000)
		{
			count++;
			outfile << line2 << endl;
			a1 = a2;
			a3 = a4;
		}
	}
	in.close();
	outfile.flush();
	outfile.close();
	return count;
}