本程序输入参数为视频文件路径，输出video轨迹数据和gyro数据

video_motion_tracker.cpp
包涵主函数

get_video_frame.cpp
提取视频文件的每一帧，提取画面中的gyro水印数据
get_video_frame.h中：
#define EXTRACT_WATERMARKS 0 //是否提取水印数据

do_sift_ransac.cpp
对视频帧进行SIFT检测，求得video轨迹
do_sift_ransac.h
#define SAVE_SIFT_IMAGE 0  //保存SIFT匹配点连线图
#define DRAW_RESULT_NO_RANSAC 0  //保存未经过RANSAC去噪的连线图
#define SET_ROI 0  //设置不检查SIFT匹配点的区域
#define TEN_PIECE 1  //是否分10块检测SIFT